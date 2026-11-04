// Mission flow: setup, choosing which trail to inspect next, and finishing at home.
#include "trail_decision_making/mission_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace trail_decision_making
{

    using namespace std::placeholders;    // for using _1, _2, etc. in std::bind
    using namespace std::chrono_literals; // for using 1s, 500ms, etc. for durations

    MissionManager::MissionManager(const rclcpp::NodeOptions &options)
        : rclcpp::Node("mission_manager", options)
    {
        // Declare parameters and get their values
        waypoints_file_ = declare_parameter<std::string>("waypoints_file", "");
        recorded_waypoints_file_ = waypoints_file_;
        phase1_waypoints_file_ = declare_parameter<std::string>("phase1_waypoints_file", "");
        global_frame_ = declare_parameter<std::string>("global_frame", "map");
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
        nav_action_name_ = declare_parameter<std::string>(
            "nav_action_name", "navigate_to_pose");
        max_goal_retries_ = declare_parameter<int>("max_goal_retries", 3);
        retry_delay_sec_ = declare_parameter<double>("retry_delay_sec", 2.0);
        waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 0.4);
        follow_trail_line_ = declare_parameter<bool>("follow_trail_line", true);
        obstacle_wait_sec_ = declare_parameter<double>("obstacle_wait_sec", 4.0);
        auto_start_ = declare_parameter<bool>("auto_start", false);
        home_x_ = declare_parameter<double>("home_x", 6.5);
        home_y_ = declare_parameter<double>("home_y", 4.0);
        // true = mark the trail blocked and go back to the previous junction; false = let Nav2 plan around.
        // Read each time an obstacle is handled, so it can be changed live with `ros2 param set`.
        declare_parameter<bool>("backtrack_on_obstacle", true);
        go_around_ahead_ = declare_parameter<double>("go_around_ahead", 2.5);
        go_around_timeout_sec_ = declare_parameter<double>("go_around_timeout_sec", 10.0);

        // Initialize TF buffer and listener, and the navigation action client
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, nav_action_name_);
        follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/husky1/obstacle_stop", 10,
            std::bind(&MissionManager::obstacleStopCallback, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to /husky1/obstacle_stop");

        // Latched so the GUI gets the current status even if it starts after us
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/husky1/mission_status", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        progress_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/husky1/mission_progress", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        // The trail network as loaded, with each trail's status, so the GUI draws exactly what we drive (latched)
        graph_pub_ = create_publisher<std_msgs::msg::String>(
            "/husky1/mission_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        // Published continuously (water_guard ignores it once it goes stale), so no state change is missed
        follow_trail_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/follow_trail", 10);
        // Where trails were found blocked, for the GUI map (JSON list, latched)
        blocked_points_pub_ = create_publisher<std_msgs::msg::String>(
            "/husky1/blocked_points", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        blocked_marker_ahead_ = declare_parameter<double>("blocked_marker_ahead", 1.2);
        inaccessible_min_gap_ = declare_parameter<double>("inaccessible_min_gap", 2.0);
        follow_trail_timer_ = create_wall_timer(200ms, [this]()
                                                {
            std_msgs::msg::Bool msg;
            msg.data = followingTrail();
            follow_trail_pub_->publish(msg); });
        // The mission is held until the GUI's start button calls this service (unless auto_start is set)
        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/husky1/start_mission",
            std::bind(&MissionManager::onStartRequest, this, _1, _2));
        // The GUI's mission controls: e-stop where we are, carry on from there, or skip what's left and go home
        pause_srv_ = create_service<std_srvs::srv::Trigger>(
            "/husky1/pause_mission",
            std::bind(&MissionManager::onPauseRequest, this, _1, _2));
        resume_srv_ = create_service<std_srvs::srv::Trigger>(
            "/husky1/resume_mission",
            std::bind(&MissionManager::onResumeRequest, this, _1, _2));
        home_srv_ = create_service<std_srvs::srv::Trigger>(
            "/husky1/return_home",
            std::bind(&MissionManager::onReturnHomeRequest, this, _1, _2));
        // The GUI's obstacle mode toggle: true = Nav2 plans around obstacles, false = backtrack (the default)
        obstacle_mode_srv_ = create_service<std_srvs::srv::SetBool>(
            "/husky1/set_obstacle_mode",
            std::bind(&MissionManager::onObstacleModeRequest, this, _1, _2));
        // The GUI's trail source toggle: true = the trails Phase 1 recorded, false = the pre-recorded ones
        trail_source_srv_ = create_service<std_srvs::srv::SetBool>(
            "/husky1/use_phase1_trails",
            std::bind(&MissionManager::onTrailSourceRequest, this, _1, _2));

        loadGraph(); // so the GUI can show the trail network before the mission starts
        publishGraph();

        if (auto_start_)
        {
            publishStatus("Waiting to start");
            start_timer_ = create_wall_timer(1s, [this]()
                                             { startMission(); });
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Holding: call /husky1/start_mission to start the inspection");
            publishStatus("Ready - waiting for the start command");
        }
    }

    // Length (m) of the shortest unblocked trail route from node_id to the home junction; infinity if there is none
    double MissionManager::distanceHomeFrom(int node_id) const
    {
        if (node_id == home_node_id_)
        {
            return 0.0;
        }
        const auto plan = planner_.planToNode(node_id, home_node_id_);
        if (plan.empty())
        {
            return std::numeric_limits<double>::infinity();
        }
        double length = 0.0;
        for (int edge_id : plan)
        {
            const TrailEdge *edge = graph_.getEdge(edge_id);
            for (size_t i = 1; edge && i < edge->waypoints.size(); ++i)
            {
                length += std::hypot(edge->waypoints[i].x - edge->waypoints[i - 1].x,
                                     edge->waypoints[i].y - edge->waypoints[i - 1].y);
            }
        }
        return length;
    }

    // Load the trail graph from the YAML file specified in the waypoints_file_ parameter.
    void MissionManager::loadGraph()
    {
        RCLCPP_INFO(get_logger(), "Loading trail graph from '%s'", waypoints_file_.c_str());

        if (!graph_.loadFromYaml(waypoints_file_))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load trail graph from '%s'", waypoints_file_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Loaded graph: %zu nodes, %zu edges",
                    graph_.nodes().size(), graph_.edges().size());
    }

    // This method is called when we want to start the mission. It checks if the graph is loaded,
    // if TF is available, and if so, it sets the current node to the nearest node to the robot's
    // current pose and starts the mission.
    void MissionManager::startMission()
    {
        RCLCPP_INFO(get_logger(), "Starting mission with frames global='%s', robot='%s'",
                    global_frame_.c_str(), robot_frame_.c_str());

        loadGraph(); // load the graph from the waypoints file
        if (graph_.edges().empty())
        {
            RCLCPP_ERROR(get_logger(), "Mission cannot start: graph contains no edges");
            publishStatus("Cannot start: trail graph has no edges");
            return;
        }
        // Check if TF is available before starting the mission
        if (!tf_buffer_->canTransform(global_frame_, robot_frame_, tf2::TimePointZero, 15s))
        {
            RCLCPP_WARN(get_logger(), "Waiting for TF: %s -> %s",
                        global_frame_.c_str(), robot_frame_.c_str());
            publishStatus("Waiting for localisation (" + global_frame_ + " -> " + robot_frame_ + ")");
            return;
        }
        // Get the robot's current pose in the global frame
        geometry_msgs::msg::TransformStamped transform;
        try // try to get the transform from the robot frame to the global frame
        {
            transform = tf_buffer_->lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex) // catch any TF exceptions and log an error
        {
            RCLCPP_ERROR(get_logger(), "Could not get robot pose: %s", ex.what());
            return;
        }

        start_timer_->cancel(); // cancel the timer so we don't try to start the mission again
        RCLCPP_INFO(get_logger(), "TF is available; starting mission");
        publishStatus("Mission started");
        current_node_id_ = graph_.findNearestNode(
            transform.transform.translation.x,
            transform.transform.translation.y);
        home_node_id_ = graph_.findNearestNode(home_x_, home_y_);
        publishProgress();
        mission_active_ = true;      // set the mission as active so we can start traversing edges
        chooseNextEdgeAndSendGoal(); // choose the next edge to traverse and send the first waypoint of that edge
    }

    void MissionManager::chooseNextEdgeAndSendGoal()
    {
        if (!mission_active_) // if the mission is not active, we don't do anything
        {
            return;
        }

        if (route_.empty()) // if we have no more edges to traverse
        {
            // Plan a route to the next uninspected edge from the current node, or home once there are none left.
            // Replanned from scratch every time, so trails blocked since the last plan are avoided.
            auto plan = returning_home_ ? planner_.planToNode(current_node_id_, home_node_id_)
                                        : planner_.planToNextUninspected(current_node_id_);
            if (plan.empty() && returning_home_ && current_node_id_ != home_node_id_)
            {
                // Every trail route home is blocked. Still go home along the trails (never straight across
                // country, e.g. over the pond): take the route through the fewest blocked trails and let Nav2
                // try to get around the obstacles on them.
                plan = planner_.planToNodeThroughBlocked(current_node_id_, home_node_id_);
                if (plan.empty())
                {
                    RCLCPP_ERROR(get_logger(), "%s is not connected to %s by any trail; stopping here",
                                 junctionName(current_node_id_).c_str(), junctionName(home_node_id_).c_str());
                    publishStatus("Stopped: no trail leads home from " + junctionName(current_node_id_) + " - " +
                                  progressText());
                    mission_active_ = false;
                    return;
                }
                RCLCPP_WARN(get_logger(), "Every trail home from %s is blocked; going home through the fewest "
                                          "blocked trails, with Nav2 going around the obstacles",
                            junctionName(current_node_id_).c_str());
                publishStatus("All trails home are blocked: trying the least blocked one");
            }
            if (plan.empty())
            {
                if (returning_home_) // at the home junction, drive the last bit to the home pose
                {
                    sendHomeGoal();
                }
                else // no reachable uninspected edges left
                {
                    startReturnHome();
                }
                return;
            }
            route_.assign(plan.begin(), plan.end()); // assign the planned route to the route_ deque so we can traverse it
        }

        current_edge_id_ = route_.front();
        route_.pop_front();                                       // remove the edge from the route so we don't traverse it again
        const TrailEdge *edge = graph_.getEdge(current_edge_id_); // get the edge from the graph so we can traverse it
        if (!edge)
        {
            RCLCPP_ERROR(get_logger(), "Could not find selected edge %d in graph", current_edge_id_);
            mission_active_ = false; // this is a fatal error, so we stop the mission
            return;
        }
        // Determine if we are traversing the edge in reverse (from to_node to from_node) or not (from from_node to to_node)
        traversing_reverse_ = (edge->to_node == current_node_id_);        // if the current node is the to_node, we are traversing in reverse
        // Coming into a blocked edge from its unchecked far side: inspect it up to the obstacle from this end
        checking_far_side_ = (edge->status == EdgeStatus::BLOCKED && edge->far_side_node == current_node_id_);
        inspecting_edge_ = (edge->status == EdgeStatus::UNINSPECTED) || checking_far_side_; // false for already-inspected edges we are only passing through
        startEdgeFollow(*edge, traversing_reverse_); // start traversing the edge, either in reverse or not, depending on the current node
    }


    // This method is called when we have finished traversing an edge, either because we reached the last waypoint
    // (or finished backtracking, or reached home). It updates the current node and edge state, and chooses the next edge to traverse.
    void MissionManager::onEdgeFinished()
    {
        if (driving_to_home_pose_) // reached the home position, nothing left to do
        {
            finishMission();
            return;
        }
        if (backtracking_) // back at the junction we started the blocked edge from; current_node_id_ is still that junction
        {
            backtracking_ = false;
            RCLCPP_INFO(get_logger(), "Back at %s; replanning around blocked edge %d",
                        junctionName(current_node_id_).c_str(), current_edge_id_);
            publishStatus("Back at " + junctionName(current_node_id_) + "; replanning - " + progressText());
            chooseNextEdgeAndSendGoal();
            return;
        }

        const TrailEdge *edge = graph_.getEdge(current_edge_id_); // get the edge again in case it was modified
        if (!edge)                                                // this should never happen, but just in case
        {
            RCLCPP_ERROR(get_logger(), "Could not finish missing edge %d", current_edge_id_);
            mission_active_ = false; // this is a fatal error, so we stop the mission
            return;
        }

        // One pass along an uninspected edge inspects it. We then carry on from the far junction rather than
        // driving back, so the next uninspected edge is planned from wherever this one ended.
        if (inspecting_edge_)
        {
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::INSPECTED); // mark the edge as inspected
            graph_.setFarSideNode(current_edge_id_, -1);
            if (checking_far_side_) // came all the way through from the other side: the obstacle has gone
            {
                const auto removed = std::remove_if(blocked_points_.begin(), blocked_points_.end(),
                                                    [this](const BlockedPoint &b) { return b.edge == current_edge_id_; });
                blocked_points_.erase(removed, blocked_points_.end());
                publishBlockedPoints();
                RCLCPP_INFO(get_logger(), "Edge %d is clear from the other side: the obstacle has gone", current_edge_id_);
            }
            publishProgress();
            publishStatus(trailName(current_edge_id_) + " inspected" +
                          (checking_far_side_ ? " (obstacle gone)" : "") + " - " + progressText());
            inspecting_edge_ = false;
            checking_far_side_ = false;
        }

        current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_); // update the current node to the other end of the edge so we can plan to the next uninspected edge
        chooseNextEdgeAndSendGoal();                                              // plan to the next uninspected edge and send the first waypoint of that edge
    }

    // No reachable uninspected trail is left, so follow the trails back to the junction nearest home
    // (see chooseNextEdgeAndSendGoal), then sendHomeGoal() drives the last bit to the home position.
    void MissionManager::startReturnHome()
    {
        returning_home_ = true;
        const size_t unreachable = graph_.countEdges(EdgeStatus::UNINSPECTED) + graph_.pendingFarSides();
        RCLCPP_INFO(get_logger(), "Inspection finished (%s, %zu unreachable); returning home to (%.2f, %.2f)",
                    progressText().c_str(), unreachable, home_x_, home_y_);
        publishStatus("Inspection finished - " + progressText() +
                      (unreachable > 0 ? ", " + std::to_string(unreachable) + " unreachable" : "") +
                      ". Returning home");
        chooseNextEdgeAndSendGoal();
    }

    void MissionManager::sendHomeGoal()
    {
        driving_to_home_pose_ = true;
        current_edge_id_ = -1;
        current_waypoints_ = {Waypoint{home_x_, home_y_, 0.0}};
        current_waypoint_index_ = 0;
        transit_to_edge_start_ = false;
        goal_retry_count_ = 0;
        sendNextWaypoint();
    }

    void MissionManager::finishMission()
    {
        driving_to_home_pose_ = false;
        returning_home_ = false;
        mission_active_ = false;
        const size_t blocked = graph_.countEdges(EdgeStatus::BLOCKED);
        // uninspected trails, and blocked trails whose far side couldn't be reached
        const size_t unreachable = graph_.countEdges(EdgeStatus::UNINSPECTED) + graph_.pendingFarSides();
        const size_t cut_off = graph_.countInaccessibleSections(); // blocked at both ends
        RCLCPP_INFO(get_logger(), "Mission complete: %s, %zu blocked (%zu at both ends), %zu unreachable; robot is home",
                    progressText().c_str(), blocked, cut_off, unreachable);
        publishProgress();
        publishStatus("Mission complete - " + progressText() + " (" +
                      std::to_string(graph_.countEdges(EdgeStatus::INSPECTED)) + " inspected, " +
                      std::to_string(blocked) + " blocked, " + std::to_string(unreachable) +
                      (home_requested_ ? " skipped" : " unreachable") +
                      (cut_off > 0 ? ", " + std::to_string(cut_off) + " cut off between two obstacles" : "") +
                      "). Robot is home");
    }

} // namespace trail_decision_making
