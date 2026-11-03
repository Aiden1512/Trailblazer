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

    // Called when the operator presses "Start Autonomous Inspection" in the GUI. Checks the
    // trail graph can be loaded, then hands over to the start timer, which retries startMission()
    // every second until localisation is available. Progress after this goes out on mission_status.
    void MissionManager::onStartRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (mission_active_)
        {
            response->success = false;
            response->message = "Mission is already running";
            return;
        }
        if (start_timer_ && !start_timer_->is_canceled())
        {
            response->success = false;
            response->message = "Mission is already starting";
            return;
        }
        loadGraph(); // check the graph now so a bad waypoints file is reported straight back to the GUI
        if (graph_.edges().empty())
        {
            response->success = false;
            response->message = "Cannot start: no trail graph loaded from '" + waypoints_file_ + "'";
            publishStatus("Cannot start: trail graph has no edges");
            return;
        }

        // Clear anything left over from a previous (finished or stopped) run
        route_.clear();
        current_goal_handle_.reset();
        follow_goal_handle_.reset();
        line_goal_sent_ = false;
        going_around_ = false;
        inspecting_edge_ = false;
        backtracking_ = false;
        returning_home_ = false;
        driving_to_home_pose_ = false;
        paused_ = false;
        home_requested_ = false;
        failed_recoveries_ = 0;
        checking_far_side_ = false;
        blocked_points_.clear();
        publishBlockedPoints();

        RCLCPP_INFO(get_logger(), "Start command received");
        publishStatus("Start command received");
        start_timer_ = create_wall_timer(1s, [this]()
                                         { startMission(); });
        response->success = true;
        response->message = "Mission starting (" + std::to_string(graph_.edges().size()) + " trail edges)";
    }

    // The GUI's E-STOP: cancel whatever Nav2 is doing so the robot stops where it is, and send nothing more
    // until Resume or Return Home. The mission state (edge, waypoint, route) is kept as it was.
    void MissionManager::onPauseRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No mission is running";
            return;
        }
        if (paused_)
        {
            response->success = false;
            response->message = "Mission is already stopped";
            return;
        }
        paused_ = true;
        haltRobot();
        RCLCPP_WARN(get_logger(), "E-STOP from the GUI: robot stopped, mission paused");
        publishStatus("E-STOP: robot stopped - " + progressText() + ". Resume or Return Home to carry on");
        response->success = true;
        response->message = "E-stop: robot stopped";
    }

    void MissionManager::onResumeRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No mission is running";
            return;
        }
        if (!paused_)
        {
            response->success = false;
            response->message = "Mission is not stopped";
            return;
        }
        RCLCPP_INFO(get_logger(), "Resume from the GUI");
        publishStatus("Resuming mission - " + progressText());
        resumeDriving();
        response->success = true;
        response->message = "Mission resumed";
    }

    // The GUI's obstacle mode toggle. Only sets backtrack_on_obstacle, which is read each time an obstacle is
    // handled, so it applies from the next obstacle (or the one being waited on). Going around, if Nav2 still
    // can't get past, the trail is marked blocked and we backtrack anyway (handleNavigationFailure).
    void MissionManager::onObstacleModeRequest(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        set_parameter(rclcpp::Parameter("backtrack_on_obstacle", !request->data));
        response->success = true;
        response->message = request->data
                                ? "Obstacle mode: Nav2 plans around obstacles (backtracks only if it can't get past)"
                                : "Obstacle mode: mark the trail blocked and backtrack to the previous junction";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    // The GUI's Return Home: skip every trail still to inspect and follow the trails home from here.
    // Also works while e-stopped (it resumes, heading home).
    void MissionManager::onReturnHomeRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No mission is running";
            return;
        }
        if (returning_home_) // already on the way (all inspected, or asked before): just make sure we're moving
        {
            response->success = true;
            response->message = paused_ ? "Resuming: already heading home" : "Already heading home";
            if (paused_)
            {
                publishStatus("Resuming: heading home - " + progressText());
                resumeDriving();
            }
            return;
        }

        haltRobot(); // stop the current goal; one heading home replaces it below
        clearPause();
        if (line_goal_sent_)
        {
            updateWaypointIndexFromPose(); // how far along the trail the robot got
            line_goal_sent_ = false;
        }
        const size_t skipped = graph_.countEdges(EdgeStatus::UNINSPECTED) + graph_.pendingFarSides();
        returning_home_ = true;
        home_requested_ = true;
        route_.clear(); // chooseNextEdgeAndSendGoal() plans home instead
        RCLCPP_WARN(get_logger(), "Return home from the GUI: skipping %zu trails still to inspect", skipped);
        publishStatus("Return home requested: skipping " + std::to_string(skipped) + " trails still to inspect");
        response->success = true;
        response->message = "Returning home (skipping " + std::to_string(skipped) + " trails)";

        if (backtracking_ || driving_to_home_pose_)
        {
            sendNextWaypoint(); // already heading for a junction (or home itself); the plan home goes on from there
            return;
        }
        if (!onTrailEdge()) // off the network, driving to a trail's start: go home from the nearest junction
        {
            replanFromNearestJunction();
            return;
        }

        // Part way along a trail: finish it or turn back, whichever end gets us home sooner along the trails.
        // A blocked trail can't be driven through, so turn back from that one.
        const auto &w = current_waypoints_;
        const size_t k = std::clamp<size_t>(current_waypoint_index_, 1, w.size());
        double back = 0.0, ahead = 0.0;
        for (size_t i = 1; i < w.size(); ++i)
        {
            (i < k ? back : ahead) += std::hypot(w[i].x - w[i - 1].x, w[i].y - w[i - 1].y);
        }
        back += distanceHomeFrom(current_node_id_);
        ahead = graph_.getEdgeStatus(current_edge_id_) == EdgeStatus::BLOCKED
                    ? std::numeric_limits<double>::infinity()
                    : ahead + distanceHomeFrom(planner_.otherEnd(current_edge_id_, current_node_id_));
        going_around_ = false;
        checking_far_side_ = false;
        if (back <= ahead)
        {
            inspecting_edge_ = false; // turning back, so the trail stays uninspected
            RCLCPP_INFO(get_logger(), "Turning back along edge %d to %s to head home", current_edge_id_,
                        junctionName(current_node_id_).c_str());
            driveBackToJunction();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Finishing edge %d, then heading home", current_edge_id_);
            sendNextWaypoint();
        }
    }

    // Stop the robot where it is: cancel the Nav2 goals and anything waiting to send a new one
    void MissionManager::haltRobot()
    {
        if (retry_timer_)
        {
            retry_timer_->cancel();
        }
        if (obstacle_wait_timer_)
        {
            obstacle_wait_timer_->cancel();
        }
        obstacle_pause_active_ = false;
        cancelGoals();
    }

    // Leave the e-stop. Forgets any obstacle seen before or during the stop, so the next obstacle_stop message
    // (sent every lidar frame) stops us for it again rather than driving into it.
    void MissionManager::clearPause()
    {
        paused_ = false;
        if (!obstacle_avoidance_active_) // going around, it is ignored until it clears anyway
        {
            obstacle_stop_active_ = false;
        }
        goal_retry_count_ = 0;
    }

    // Carry on after an e-stop from where the robot is (a trail line is resent from the nearest waypoint).
    void MissionManager::resumeDriving()
    {
        clearPause();
        sendNextWaypoint();
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
    // Follow the edge's waypoints from current_node_id_ to its other end, driving to the entry waypoint first if we are not there.
    void MissionManager::startEdgeFollow(const TrailEdge &edge, bool reverse)
    {
        auto waypoints = edge.waypoints; // copy the waypoints so we can modify them without affecting the original edge
        if (waypoints.empty())
        {
            RCLCPP_ERROR(get_logger(), "Edge %d has no waypoints; cannot navigate", edge.id);
            mission_active_ = false;
            return;
        }
        // if we are traversing the edge in reverse, we need to reverse the order of the waypoints and adjust their yaw angles
        if (reverse)
        {
            std::reverse(waypoints.begin(), waypoints.end());
            // adjust the yaw angles of the waypoints so they are facing the correct direction when traversing in reverse
            for (auto &waypoint : waypoints)
            {
                waypoint.yaw = std::atan2(
                    std::sin(waypoint.yaw + M_PI),
                    std::cos(waypoint.yaw + M_PI));
            }
        }

        // Drop near-duplicate waypoints (closer than waypoint_spacing_ to the last one kept), keeping the first and last.
        // Recorded and hand-added waypoints are ~1 m apart and each one keeps Nav2's planner on the trail, so
        // only duplicates go: a wider spacing (it was 1.2 m) threw away real waypoints and let the planner take
        // its own way between the ones left, e.g. a U-turn round the tree at the start of trail 2.
        std::vector<Waypoint> thinned;
        thinned.push_back(waypoints.front());
        for (size_t index = 1; index < waypoints.size(); ++index)
        {
            const double dx = waypoints[index].x - thinned.back().x;
            const double dy = waypoints[index].y - thinned.back().y;
            if (std::hypot(dx, dy) >= waypoint_spacing_)
            {
                thinned.push_back(waypoints[index]);
            }
        }
        // always include the last waypoint, even if it is close to the previous one, to ensure we reach the end of the edge
        if (std::hypot(waypoints.back().x - thinned.back().x,
                       waypoints.back().y - thinned.back().y) > 0.05)
        {
            thinned.push_back(waypoints.back());
        }

        current_waypoints_ = thinned;
        size_t first_waypoint = 0;
        transit_to_edge_start_ = false;
        try
        {
            const auto pose = tf_buffer_->lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            const double dx = thinned.front().x - pose.transform.translation.x;
            const double dy = thinned.front().y - pose.transform.translation.y;
            const double distance_to_entry = std::hypot(dx, dy);
            if (distance_to_entry < 0.5 && current_waypoints_.size() > 1)
            {
                first_waypoint = 1;
                RCLCPP_INFO(get_logger(),
                            "Skipping edge %d start waypoint; robot is %.2f m from it",
                            edge.id, distance_to_entry);
            }
            else if (distance_to_entry >= 0.5)
            {
                transit_to_edge_start_ = true;
                RCLCPP_INFO(get_logger(),
                            "Transiting %.2f m to edge %d entry waypoint (%.2f, %.2f)",
                            distance_to_entry, edge.id,
                            thinned.front().x, thinned.front().y);
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not check current pose before navigation: %s", ex.what());
        }

        goal_retry_count_ = 0;
        current_waypoint_index_ = first_waypoint; // set the current waypoint index to the first waypoint we want to navigate to
        sendNextWaypoint();                       // send the first waypoint to Nav2 to start navigating the edge
    }

    // send the next waypoint in the current edge to Nav2 for navigation.
    // If we have reached the last waypoint, we call onEdgeFinished() to handle the end of the edge traversal.
    void MissionManager::sendNextWaypoint()
    {
        if (paused_) // every goal goes out through here, so nothing moves the robot until resumed
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Mission e-stopped: not sending goals");
            return;
        }
        if (obstacle_pause_active_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Navigation paused: obstacle is inside the stop range");
            return;
        }
        if (followTrailLine())
        {
            sendTrailLine();
            return;
        }
        if (current_waypoint_index_ >= current_waypoints_.size())
        {
            onEdgeFinished();
            return;
        }
        if (!nav_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 action server '%s' is not available",
                         nav_action_name_.c_str());
            publishStatus("Stopped: Nav2 is not available");
            mission_active_ = false;
            return;
        }
        const auto &waypoint = current_waypoints_[current_waypoint_index_];
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get pose before waypoint: %s", ex.what());
            return;
        }

        const double dx = waypoint.x - transform.transform.translation.x;
        const double dy = waypoint.y - transform.transform.translation.y;
        const double target_yaw = std::atan2(dy, dx);

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = global_frame_;
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = waypoint.x;
        goal.pose.pose.position.y = waypoint.y;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, target_yaw);
        goal.pose.pose.orientation = tf2::toMsg(orientation);

        if (driving_to_home_pose_)
        {
            RCLCPP_INFO(get_logger(), "Sending home goal (%.2f, %.2f)", waypoint.x, waypoint.y);
            publishStatus("Returning to the home position - " + progressText());
        }
        else if (backtracking_)
        {
            RCLCPP_INFO(get_logger(), "Backtracking to %s: waypoint %zu/%zu (%.2f, %.2f)",
                        junctionName(current_node_id_).c_str(), current_waypoint_index_ + 1,
                        current_waypoints_.size(), waypoint.x, waypoint.y);
            publishStatus((home_requested_ ? "Heading home: back to " : trailName(current_edge_id_) + " blocked: returning to ") +
                          junctionName(current_node_id_) + " (" + std::to_string(current_waypoint_index_ + 1) +
                          "/" + std::to_string(current_waypoints_.size()) + ")");
        }
        else if (transit_to_edge_start_ && current_waypoint_index_ == 0)
        {
            RCLCPP_INFO(get_logger(), "Sending transit goal to edge %d entry waypoint",
                        current_edge_id_);
            publishStatus("Driving to the start of " + trailName(current_edge_id_, false));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Sending edge %d waypoint %zu/%zu: (%.2f, %.2f)",
                        current_edge_id_, current_waypoint_index_ + 1,
                        current_waypoints_.size(), waypoint.x, waypoint.y);
            publishStatus(std::string(returning_home_ ? "Heading home - " : "") +
                          trailName(current_edge_id_) +
                          (checking_far_side_ ? " (checking the far side of the obstacle)"
                                              : inspecting_edge_ ? " (inspecting)" : "") + ": waypoint " +
                          std::to_string(current_waypoint_index_ + 1) + "/" +
                          std::to_string(current_waypoints_.size()));
        }

        // Send the goal to Nav2 and set up callbacks for feedback and result
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        const unsigned seq = ++goal_seq_;
        options.goal_response_callback = [this, seq](
                                             const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            if (seq != goal_seq_) // cancelled (e-stop, obstacle) or replaced before Nav2 accepted it
            {
                if (goal_handle)
                {
                    nav_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "Nav2 rejected the navigation goal");
                publishStatus("Stopped: Nav2 rejected the goal");
                mission_active_ = false;
                return;
            }
            current_goal_handle_ = goal_handle;
        };
        // send the goal to Nav2 asynchronously so we can continue processing other events while Nav2 is navigating
        options.feedback_callback = [this, seq](GoalHandleNavigateToPose::SharedPtr handle,
                                                const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            if (seq == goal_seq_)
            {
                onNavigationFeedback(handle, feedback);
            }
        };
        options.result_callback = [this, seq](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            if (seq != goal_seq_)
            {
                RCLCPP_DEBUG(get_logger(), "Ignoring the result of a superseded NavigateToPose goal");
                return;
            }
            onNavigationResult(result);
        };
        nav_client_->async_send_goal(goal, options);
    }

    // This method is called when we receive the result of the navigation goal from Nav2.
    // It checks if the navigation was successful, and if so, it either sends the next waypoint
    // or calls onEdgeFinished() if we have reached the last waypoint of the edge.
    void MissionManager::onNavigationResult(
        const GoalHandleNavigateToPose::WrappedResult &result)
    {
        // Check if the navigation was successful
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            handleGoalFailure(result.code);
            return;
        }

        goal_retry_count_ = 0;
        failed_recoveries_ = 0;
        transit_to_edge_start_ = false;
        going_around_ = false; // reached the waypoint we were going round an obstacle to
        ++current_waypoint_index_;
        // If we have not reached the last waypoint, send the next waypoint to Nav2.
        if (current_waypoint_index_ < current_waypoints_.size())
        {
            sendNextWaypoint();
        }
        else // we have reached the last waypoint, call onEdgeFinished() to handle the end of the edge traversal.
        {
            RCLCPP_INFO(get_logger(), "Nav2 completed edge %d", current_edge_id_);
            onEdgeFinished();
        }
    }

    // A Nav2 goal (NavigateToPose or FollowPath) didn't succeed: ignore cancels (we cancel goals ourselves and
    // whoever cancelled resends), retry aborts a few times, then treat the edge as impassable.
    void MissionManager::handleGoalFailure(rclcpp_action::ResultCode code)
    {
        {
            // We only cancel goals ourselves (obstacle pause), and whoever cancelled
            // is responsible for resending, so a cancelled result is never a failure.
            if (code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_INFO(get_logger(), "Nav2 goal cancelled");
                return;
            }
            // An abort is often transient (e.g. stale costmap cells around the robot
            // making the planner fail), so resend the same waypoint a few times
            // before giving up on the edge.
            if (code == rclcpp_action::ResultCode::ABORTED &&
                goal_retry_count_ < max_goal_retries_)
            {
                ++goal_retry_count_;
                RCLCPP_WARN(get_logger(),
                            "Nav2 aborted edge %d waypoint %zu; retrying in %.1f s (%d/%d)",
                            current_edge_id_, current_waypoint_index_ + 1,
                            retry_delay_sec_, goal_retry_count_, max_goal_retries_);
                publishStatus("Nav2 aborted; retrying waypoint (" + std::to_string(goal_retry_count_) +
                              "/" + std::to_string(max_goal_retries_) + ")");
                retry_timer_ = create_wall_timer(
                    std::chrono::duration<double>(retry_delay_sec_), [this]()
                    {
                        retry_timer_->cancel();
                        if (mission_active_ && !obstacle_pause_active_)
                        {
                            sendNextWaypoint();
                        } });
                return;
            }
            RCLCPP_WARN(get_logger(),
                        "Nav2 failed edge %d: action result code=%d",
                        current_edge_id_, static_cast<int>(code));
            handleNavigationFailure();
        }
    }

    // Trail edges (inspecting, passing through, backtracking) are driven by handing Nav2's controller (RPP) the
    // recorded trail line itself. The robot drove these lines when the trails were mapped, but low branches put
    // costmap obstacles within centimetres of them, so the global planner would route round (U-turns, swinging
    // onto other trails). The planner is still used to get onto the network, round obstacles, and to the home pose.
    bool MissionManager::followTrailLine() const
    {
        return follow_trail_line_ && current_edge_id_ >= 0 && !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0) && !going_around_;
    }

    // After a trail-line goal was interrupted (obstacle, abort), move current_waypoint_index_ on to the first
    // waypoint the robot hasn't passed yet: the end of the line segment it is closest to.
    void MissionManager::updateWaypointIndexFromPose()
    {
        const auto &w = current_waypoints_;
        if (w.size() < 2 || current_waypoint_index_ >= w.size())
        {
            return;
        }
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &)
        {
            return;
        }
        const double x = tf.transform.translation.x, y = tf.transform.translation.y;
        size_t best = current_waypoint_index_;
        double best_d = std::numeric_limits<double>::max();
        for (size_t i = current_waypoint_index_ > 0 ? current_waypoint_index_ - 1 : 0; i + 1 < w.size(); ++i)
        {
            const double dx = w[i + 1].x - w[i].x, dy = w[i + 1].y - w[i].y;
            const double len2 = dx * dx + dy * dy;
            const double t = len2 > 1e-9 ? std::clamp(((x - w[i].x) * dx + (y - w[i].y) * dy) / len2, 0.0, 1.0) : 0.0;
            const double d = std::hypot(x - w[i].x - t * dx, y - w[i].y - t * dy);
            if (d < best_d)
            {
                best_d = d;
                best = i + 1;
            }
        }
        if (best < w.size() && std::hypot(x - w[best].x, y - w[best].y) < 0.3)
        {
            ++best; // right on that waypoint: it counts as reached
        }
        current_waypoint_index_ = std::max(current_waypoint_index_, best);
    }

    // Going round an obstacle: the next waypoint is usually less than a metre ahead, short of the obstacle (which
    // is about blocked_marker_ahead_ in front when we stop), so Nav2 would count it reached at once and the trail
    // line would lead straight back into the obstacle. Aim for the first waypoint at least go_around_ahead_
    // further along the trail instead, so the planner has to take us past it.
    void MissionManager::skipWaypointsPastObstacle()
    {
        auto &w = current_waypoints_;
        if (driving_to_home_pose_ || (transit_to_edge_start_ && current_waypoint_index_ == 0) ||
            current_waypoint_index_ >= w.size())
        {
            return;
        }
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &)
        {
            return;
        }
        size_t i = current_waypoint_index_;
        double along = std::hypot(w[i].x - tf.transform.translation.x, w[i].y - tf.transform.translation.y);
        while (along < go_around_ahead_ && i + 1 < w.size())
        {
            along += std::hypot(w[i + 1].x - w[i].x, w[i + 1].y - w[i].y);
            ++i;
        }
        RCLCPP_INFO(get_logger(), "Going around: aiming for waypoint %zu/%zu, %.1f m along the trail",
                    i + 1, w.size(), along);
        current_waypoint_index_ = i;
    }

    // One FollowPath goal from the robot along all the remaining waypoints of the edge, densified to 0.1 m
    void MissionManager::sendTrailLine()
    {
        if (line_goal_sent_)
        {
            updateWaypointIndexFromPose();
        }
        if (current_waypoint_index_ >= current_waypoints_.size())
        {
            line_goal_sent_ = false;
            onEdgeFinished();
            return;
        }
        if (!follow_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 controller action server 'follow_path' is not available");
            publishStatus("Stopped: Nav2 is not available");
            mission_active_ = false;
            return;
        }
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get pose before following the trail: %s", ex.what());
            return;
        }

        FollowPath::Goal goal;
        goal.controller_id = "FollowPath";
        goal.path.header.frame_id = global_frame_;
        goal.path.header.stamp = now();
        auto add_pose = [&](double x, double y, double yaw)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = goal.path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
            goal.path.poses.push_back(pose);
        };
        double px = tf.transform.translation.x, py = tf.transform.translation.y;
        double yaw = 0.0;
        for (size_t i = current_waypoint_index_; i < current_waypoints_.size(); ++i)
        {
            const Waypoint &w = current_waypoints_[i];
            yaw = std::atan2(w.y - py, w.x - px);
            const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(w.x - px, w.y - py) / 0.1)));
            for (int s = 0; s < steps; ++s)
            {
                add_pose(px + (w.x - px) * s / steps, py + (w.y - py) * s / steps, yaw);
            }
            px = w.x;
            py = w.y;
        }
        add_pose(px, py, yaw);

        const size_t remaining = current_waypoints_.size() - current_waypoint_index_;
        if (backtracking_)
        {
            RCLCPP_INFO(get_logger(), "Backtracking to %s along the trail (%zu waypoints)",
                        junctionName(current_node_id_).c_str(), remaining);
            publishStatus((home_requested_ ? "Heading home: back to " : trailName(current_edge_id_) + " blocked: returning to ") +
                          junctionName(current_node_id_));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Following edge %d along the trail from waypoint %zu/%zu",
                        current_edge_id_, current_waypoint_index_ + 1, current_waypoints_.size());
            publishStatus(std::string(returning_home_ ? "Heading home - " : "") + trailName(current_edge_id_) +
                          (checking_far_side_ ? " (checking the far side of the obstacle)"
                                              : inspecting_edge_ ? " (inspecting)" : "") +
                          ": following the trail, " + std::to_string(remaining) + " waypoints to go");
        }

        rclcpp_action::Client<FollowPath>::SendGoalOptions options;
        const unsigned seq = ++goal_seq_;
        options.goal_response_callback = [this, seq](const GoalHandleFollowPath::SharedPtr &goal_handle)
        {
            if (seq != goal_seq_) // cancelled (e-stop, obstacle) or replaced before Nav2 accepted it
            {
                if (goal_handle)
                {
                    follow_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "Nav2 rejected the trail line");
                publishStatus("Stopped: Nav2 rejected the goal");
                mission_active_ = false;
                return;
            }
            follow_goal_handle_ = goal_handle;
        };
        options.feedback_callback = [this, seq](GoalHandleFollowPath::SharedPtr,
                                                const std::shared_ptr<const FollowPath::Feedback> feedback)
        {
            if (seq != goal_seq_)
            {
                return;
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Following edge %d: %.2f m to go along the trail, %.2f m/s",
                                 current_edge_id_, feedback->distance_to_goal, feedback->speed);
        };
        options.result_callback = [this, seq](const GoalHandleFollowPath::WrappedResult &result)
        {
            if (seq != goal_seq_)
            {
                RCLCPP_DEBUG(get_logger(), "Ignoring the result of a superseded FollowPath goal");
                return;
            }
            onTrailLineResult(result);
        };
        line_goal_sent_ = true;
        follow_client_->async_send_goal(goal, options);
    }

    void MissionManager::onTrailLineResult(const GoalHandleFollowPath::WrappedResult &result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            handleGoalFailure(result.code); // a retry resends the line from wherever the robot now is
            return;
        }
        // The goal checker passed at the end of the line: the whole edge has been driven
        line_goal_sent_ = false;
        goal_retry_count_ = 0;
        failed_recoveries_ = 0;
        transit_to_edge_start_ = false;
        current_waypoint_index_ = current_waypoints_.size();
        RCLCPP_INFO(get_logger(), "Nav2 completed edge %d along the trail", current_edge_id_);
        onEdgeFinished();
    }

    // The stored handles aren't cleared when a goal finishes, and once its result is in the action client
    // forgets it and cancelling throws UnknownGoalHandleError; that just means there is nothing left to cancel.
    void MissionManager::cancelGoals()
    {
        ++goal_seq_; // also covers a goal sent but not yet accepted: its response callback cancels it
        if (current_goal_handle_)
        {
            try
            {
                nav_client_->async_cancel_goal(current_goal_handle_);
            }
            catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
            {
                RCLCPP_DEBUG(get_logger(), "NavigateToPose goal already finished; nothing to cancel");
            }
            current_goal_handle_.reset();
        }
        if (follow_goal_handle_)
        {
            try
            {
                follow_client_->async_cancel_goal(follow_goal_handle_);
            }
            catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
            {
                RCLCPP_DEBUG(get_logger(), "FollowPath goal already finished; nothing to cancel");
            }
            follow_goal_handle_.reset();
        }
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

    // When an obstacle enters the stop range we cancel the Nav2 goal so the robot stops, and wait
    // obstacle_wait_sec_ for the obstacle to move away. If it is still there, either:
    //  - backtrack_on_obstacle: mark the edge blocked, retrace it to the junction we came from and
    //    replan the rest of the inspection from there, or
    //  - otherwise: resend the current waypoint so Nav2 plans a path around it.
    // While Nav2 is going around (or we are backtracking away from the obstacle), the stop topic is
    // ignored until it clears, as the robot will pass close to the obstacle.
    void MissionManager::obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == obstacle_stop_active_) // if the obstacle stop state has not changed, we don't need to do anything
        {
            return;
        }

        obstacle_stop_active_ = msg->data;
        if (obstacle_stop_active_)
        {
            if (obstacle_avoidance_active_ || !mission_active_ || paused_)
            {
                return; // already going around an obstacle (let Nav2 handle it), or not driving at all
            }
            RCLCPP_WARN(get_logger(),
                        "Obstacle is inside the stop range; stopping for %.1f s before %s",
                        obstacle_wait_sec_, canBacktrack() ? "backtracking" : "going around");
            publishStatus("Obstacle ahead: stopped");
            obstacle_pause_active_ = true;
            if (retry_timer_)
            {
                retry_timer_->cancel();
            }
            cancelGoals();
            obstacle_wait_timer_ = create_wall_timer(
                std::chrono::duration<double>(obstacle_wait_sec_), [this]()
                {
                    obstacle_wait_timer_->cancel();
                    obstacle_pause_active_ = false;
                    if (!mission_active_)
                    {
                        return;
                    }
                    if (canBacktrack())
                    {
                        startBacktrackToJunction("Obstacle still blocking");
                        return;
                    }
                    RCLCPP_INFO(get_logger(), "Obstacle still present; asking Nav2 to plan around it");
                    if (line_goal_sent_)
                    {
                        updateWaypointIndexFromPose(); // go round to the next waypoint the robot hasn't passed
                        line_goal_sent_ = false;
                    }
                    recordBlockedPoint(true); // still facing it: mark it on the GUI map before turning away
                    skipWaypointsPastObstacle();
                    going_around_ = true; // the planner, not the trail line, gets us past it
                    obstacle_avoidance_active_ = true;
                    goal_retry_count_ = 0;
                    sendNextWaypoint(); });
        }
        else
        {
            obstacle_avoidance_active_ = false;
            if (obstacle_pause_active_) // obstacle moved away during the wait; resume straight away
            {
                obstacle_wait_timer_->cancel();
                obstacle_pause_active_ = false;
                RCLCPP_INFO(get_logger(), "Obstacle stop cleared; resuming navigation");
                if (mission_active_)
                {
                    sendNextWaypoint();
                }
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Obstacle stop cleared");
            }
        }
    }

    // True while driving along a trail edge, i.e. there is an edge to block and a junction to go back to.
    // Not while driving to an edge's entry from off the trail network, on the final leg home, or while
    // already backtracking.
    bool MissionManager::onTrailEdge() const
    {
        return current_edge_id_ >= 0 &&
               !backtracking_ &&
               !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0);
    }

    // While true, water_guard's camera trail-keeper may steer the robot back onto the brown trail when Nav2
    // cuts a corner. Not while driving onto the network or to the home pose (both off the trail), nor while
    // stopped for or going around an obstacle (Nav2 has to leave the trail to get past it).
    bool MissionManager::followingTrail() const
    {
        return mission_active_ &&
               !paused_ &&
               current_edge_id_ >= 0 &&
               !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0) &&
               !obstacle_pause_active_ &&
               !obstacle_avoidance_active_ &&
               !going_around_;
    }

    // Obstacles are only backtracked from on a trail edge, and only in backtrack mode;
    // otherwise Nav2 goes around the obstacle instead.
    bool MissionManager::canBacktrack() const
    {
        return get_parameter("backtrack_on_obstacle").as_bool() && onTrailEdge() && !forcingBlockedEdge();
    }

    // On the way home through a trail already known to be blocked (because every route home is): backing off
    // it again would just loop, so Nav2 goes around the obstacle instead.
    bool MissionManager::forcingBlockedEdge() const
    {
        return returning_home_ && current_edge_id_ >= 0 && !backtracking_ &&
               graph_.getEdgeStatus(current_edge_id_) == EdgeStatus::BLOCKED;
    }

    // Nav2 gave up on a waypoint (after retries). Rather than ending the mission, treat the edge as
    // impassable and carry on inspecting whatever is still reachable, the long way round if need be.
    void MissionManager::handleNavigationFailure()
    {
        if (++failed_recoveries_ > max_failed_recoveries_) // stuck: every recovery since the last reached waypoint failed too
        {
            RCLCPP_ERROR(get_logger(), "Nav2 failed %d times in a row; stopping the mission", failed_recoveries_);
            publishStatus("Stopped: Nav2 keeps failing - " + progressText());
            mission_active_ = false;
            return;
        }
        if (driving_to_home_pose_)
        {
            RCLCPP_ERROR(get_logger(), "Nav2 could not reach the home position");
            publishStatus("Inspection finished - " + progressText() + ", but Nav2 could not reach the home position");
            mission_active_ = false;
            return;
        }
        if (forcingBlockedEdge()) // Nav2 couldn't get around the obstacle on the only way home either
        {
            RCLCPP_ERROR(get_logger(), "Nav2 cannot get past the obstacle on edge %d, the only trail home; stopping",
                         current_edge_id_);
            publishStatus("Stopped: every trail home is blocked - " + progressText());
            mission_active_ = false;
            return;
        }
        if (onTrailEdge()) // part way along the edge: block it and drive back to the junction we came from
        {
            startBacktrackToJunction("Nav2 cannot get along");
            return;
        }
        if (current_edge_id_ >= 0 && !backtracking_) // could not even reach the edge's entry from here, so don't try it again
        {
            markCurrentEdgeBlocked();
            RCLCPP_WARN(get_logger(), "Could not reach the start of edge %d; marked blocked", current_edge_id_);
        }
        // Could not reach the edge entry, or could not drive back to the junction: carry on from the nearest junction
        backtracking_ = false;
        replanFromNearestJunction();
    }

    void MissionManager::replanFromNearestJunction()
    {
        try
        {
            const auto pose = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            current_node_id_ = graph_.findNearestNode(pose.transform.translation.x, pose.transform.translation.y);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get robot pose to replan; using %s: %s",
                        junctionName(current_node_id_).c_str(), ex.what());
        }
        route_.clear();
        inspecting_edge_ = false;
        RCLCPP_INFO(get_logger(), "Replanning from %s", junctionName(current_node_id_).c_str());
        publishStatus("Replanning from " + junctionName(current_node_id_) + " - " + progressText());
        chooseNextEdgeAndSendGoal();
    }

    // The edge can't be driven (obstacle still there after the wait, or Nav2 gave up): mark it blocked, then
    // retrace the waypoints already driven back to the junction this traversal started from (current_node_id_).
    // When that is done onEdgeFinished() replans the rest of the inspection, which never uses blocked edges.
    void MissionManager::startBacktrackToJunction(const std::string &reason)
    {
        if (line_goal_sent_)
        {
            updateWaypointIndexFromPose(); // which waypoints were reached before the obstacle
            line_goal_sent_ = false;
        }
        going_around_ = false;
        const bool far_side_next = markCurrentEdgeBlocked();
        const double cut_off = checking_far_side_ ? flagInaccessibleSection() : 0.0;
        route_.clear(); // the old route may go through the blocked edge
        inspecting_edge_ = false;
        checking_far_side_ = false;
        RCLCPP_WARN(get_logger(), "%s edge %d; marked blocked, returning to %s%s",
                    reason.c_str(), current_edge_id_, junctionName(current_node_id_).c_str(),
                    far_side_next ? " (will check it from the other end later)" : "");
        publishStatus(trailName(current_edge_id_) + " blocked by an obstacle" +
                      (far_side_next ? "; will check it from the other end" : "") +
                      (cut_off > 0.0 ? "; blocked at both ends, so the " +
                                           std::to_string(static_cast<int>(std::round(cut_off))) +
                                           " m between the two obstacles can't be checked"
                                     : "") +
                      " - " + progressText());

        obstacle_avoidance_active_ = true; // the obstacle stays in the stop range while we turn away from it
        driveBackToJunction();
    }

    // Waypoints before current_waypoint_index_ have been reached; drive them in reverse, ending at the junction
    // this traversal started from. onEdgeFinished() then replans from there.
    void MissionManager::driveBackToJunction()
    {
        const size_t reached = std::max<size_t>(current_waypoint_index_, 1);
        std::vector<Waypoint> back(current_waypoints_.begin(), current_waypoints_.begin() + reached);
        std::reverse(back.begin(), back.end());
        current_waypoints_ = back;
        current_waypoint_index_ = 0;
        transit_to_edge_start_ = false;
        goal_retry_count_ = 0;
        backtracking_ = true;
        sendNextWaypoint();
    }

    // No reachable uninspected trail is left, so follow the trails back to the junction nearest home
    // (see chooseNextEdgeAndSendGoal), then sendHomeGoal() drives the last bit to the home position.
    // Mark the current edge blocked where the robot is now, and put a marker on the GUI map. If it was being
    // inspected for the first time, the stretch beyond the obstacle is still uninspected, so it is to be
    // inspected later from the other end (returns true); if we were already coming from that other end
    // (or only passing through), there is nothing more to check on it.
    bool MissionManager::markCurrentEdgeBlocked()
    {
        const TrailEdge *edge = graph_.getEdge(current_edge_id_);
        const bool first_time = edge && edge->status == EdgeStatus::UNINSPECTED;
        graph_.setEdgeStatus(current_edge_id_, EdgeStatus::BLOCKED);
        graph_.setFarSideNode(current_edge_id_, first_time ? planner_.otherEnd(current_edge_id_, current_node_id_) : -1);
        recordBlockedPoint();
        publishProgress();
        return first_time;
    }

    // Checking a blocked edge from its far side and stopped by an obstacle again: the robot has now been blocked
    // coming from each end, so the stretch of trail between the two obstacles can't be reached from either side.
    // Flag both obstacles and record that stretch on the edge, for the GUI. Returns its length (m), or 0 if the
    // two are the same obstacle seen from each side (recordBlockedPoint merged them, or they are very close).
    double MissionManager::flagInaccessibleSection()
    {
        const int far_end = planner_.otherEnd(current_edge_id_, current_node_id_);
        auto found_from = [this](int node)
        {
            return std::find_if(blocked_points_.begin(), blocked_points_.end(), [&](const BlockedPoint &b)
                                { return b.edge == current_edge_id_ && b.from_node == node && !b.avoided; });
        };
        const auto near = found_from(current_node_id_);
        const auto far = found_from(far_end);
        if (near == blocked_points_.end() || far == blocked_points_.end() || near == far)
        {
            RCLCPP_INFO(get_logger(), "Edge %d is blocked by the same obstacle from both ends", current_edge_id_);
            return 0.0;
        }
        double start = graph_.distanceAlongEdge(current_edge_id_, near->x, near->y);
        double end = graph_.distanceAlongEdge(current_edge_id_, far->x, far->y);
        if (start > end)
        {
            std::swap(start, end);
        }
        if (end - start < inaccessible_min_gap_)
        {
            RCLCPP_INFO(get_logger(), "Edge %d: obstacles from each end are %.1f m apart, taken to be the same one",
                        current_edge_id_, end - start);
            return 0.0;
        }
        graph_.setInaccessibleSection(current_edge_id_, graph_.edgeSection(current_edge_id_, start, end));
        near->bounds_gap = true;
        far->bounds_gap = true;
        RCLCPP_WARN(get_logger(), "Edge %d blocked at both ends: obstacles at (%.2f, %.2f) and (%.2f, %.2f), "
                                  "%.1f m between them can't be inspected",
                    current_edge_id_, far->x, far->y, near->x, near->y, end - start);
        publishBlockedPoints();
        publishGraph();
        return end - start;
    }

    // The obstacle is about blocked_marker_ahead_ in front of the robot when it stops for it. avoided: Nav2 is
    // going around it rather than the trail being blocked. An obstacle already marked on this edge (stopped for
    // again while going round, or going round failed and we backtrack) keeps its one marker; blocked wins.
    void MissionManager::recordBlockedPoint(bool avoided)
    {
        try
        {
            const auto tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            const auto &q = tf.transform.rotation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            BlockedPoint point;
            point.edge = current_edge_id_;
            point.x = tf.transform.translation.x + blocked_marker_ahead_ * std::cos(yaw);
            point.y = tf.transform.translation.y + blocked_marker_ahead_ * std::sin(yaw);
            point.from_node = current_node_id_;
            point.avoided = avoided;
            const auto same = std::find_if(blocked_points_.begin(), blocked_points_.end(), [&](const BlockedPoint &b)
                                           { return b.edge == point.edge && std::hypot(b.x - point.x, b.y - point.y) < 1.5; });
            if (same != blocked_points_.end())
            {
                same->avoided = same->avoided && avoided;
            }
            else
            {
                blocked_points_.push_back(point);
            }
            RCLCPP_INFO(get_logger(), "Edge %d %s at (%.2f, %.2f), coming from %s", point.edge,
                        avoided ? "has an obstacle (going around it)" : "blocked", point.x, point.y,
                        junctionName(point.from_node).c_str());
            publishBlockedPoints();
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get robot pose to mark where edge %d is blocked: %s",
                        current_edge_id_, ex.what());
        }
    }

    // JSON list for the GUI: [{"edge": 2, "x": -5.1, "y": -1.2, "from": "junction 3", "avoided": false,
    // "bounds_gap": false}, ...] in the map frame (metres)
    void MissionManager::publishBlockedPoints()
    {
        std::ostringstream json;
        json << std::fixed << std::setprecision(2) << "[";
        for (size_t i = 0; i < blocked_points_.size(); ++i)
        {
            const auto &b = blocked_points_[i];
            const TrailEdge *edge = graph_.getEdge(b.edge);
            json << (i ? ", " : "") << "{\"edge\": " << b.edge << ", \"trail\": " << (edge ? edge->trail_id : -1) << ", \"x\": " << b.x << ", \"y\": " << b.y
                 << ", \"from\": \"" << junctionName(b.from_node) << "\", \"avoided\": " << (b.avoided ? "true" : "false")
                 << ", \"bounds_gap\": " << (b.bounds_gap ? "true" : "false") << "}";
        }
        json << "]";
        std_msgs::msg::String msg;
        msg.data = json.str();
        blocked_points_pub_->publish(msg);
    }

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

    // Completion = trails that have been resolved (inspected, or found blocked) out of all trails
    void MissionManager::publishProgress()
    {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(graph_.explorationPercentage() * 100.0);
        progress_pub_->publish(msg);
        publishGraph(); // trail statuses change exactly when progress does
    }

    // JSON for the GUI (map frame, metres):
    // {"file": ..., "junctions": [{"node": 0, "junction": 1, "x": .., "y": ..}, ...],
    //  "trails": [{"edge": 0, "trail": 1, "from": 0, "to": 1, "status": "inspected", "far_side": -1,
    //              "waypoints": [[x, y], ...], "inaccessible": [[x, y], ...]}, ...]}
    // "junction" is -1 for a trail's dead end; status is unvisited / inspected / blocked. "inaccessible" is the
    // stretch between two obstacles of a trail blocked at both ends, empty otherwise.
    void MissionManager::publishGraph()
    {
        std::ostringstream json;
        json << std::fixed << std::setprecision(3) << "{\"file\": \"" << waypoints_file_ << "\", \"junctions\": [";
        const auto &nodes = graph_.nodes();
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            json << (i ? ", " : "") << "{\"node\": " << nodes[i].id << ", \"junction\": " << nodes[i].junction_id
                 << ", \"x\": " << nodes[i].position.x << ", \"y\": " << nodes[i].position.y << "}";
        }
        json << "], \"trails\": [";
        const auto &edges = graph_.edges();
        for (size_t i = 0; i < edges.size(); ++i)
        {
            const TrailEdge &e = edges[i];
            const char *status = e.status == EdgeStatus::INSPECTED ? "inspected"
                                 : e.status == EdgeStatus::BLOCKED ? "blocked"
                                                                   : "unvisited";
            json << (i ? ", " : "") << "{\"edge\": " << e.id << ", \"trail\": " << e.trail_id << ", \"from\": "
                 << e.from_node << ", \"to\": " << e.to_node << ", \"status\": \"" << status
                 << "\", \"far_side\": " << e.far_side_node << ", \"waypoints\": [";
            for (size_t w = 0; w < e.waypoints.size(); ++w)
            {
                json << (w ? ", " : "") << "[" << e.waypoints[w].x << ", " << e.waypoints[w].y << "]";
            }
            json << "], \"inaccessible\": [";
            for (size_t w = 0; w < e.inaccessible.size(); ++w)
            {
                json << (w ? ", " : "") << "[" << e.inaccessible[w].x << ", " << e.inaccessible[w].y << "]";
            }
            json << "]}";
        }
        json << "]}";
        std_msgs::msg::String msg;
        msg.data = json.str();
        graph_pub_->publish(msg);
    }

    // "Trail 3" (the trail's id in the YAML file, as on the GUI map); "trail 3" when capital is false
    std::string MissionManager::trailName(int edge_id, bool capital) const
    {
        const TrailEdge *edge = graph_.getEdge(edge_id);
        const int id = (edge && edge->trail_id >= 0) ? edge->trail_id : edge_id;
        return std::string(capital ? "Trail " : "trail ") + std::to_string(id);
    }

    std::string MissionManager::progressText() const
    {
        return std::to_string(static_cast<int>(std::round(graph_.explorationPercentage() * 100.0))) + "% complete";
    }

    std::string MissionManager::junctionName(int node_id) const
    {
        if (node_id < 0 || node_id >= static_cast<int>(graph_.nodes().size()))
        {
            return "unknown junction";
        }
        const int junction_id = graph_.nodes()[node_id].junction_id;
        return junction_id >= 0 ? "junction " + std::to_string(junction_id)
                                : "trail end " + std::to_string(node_id);
    }

    void MissionManager::publishStatus(const std::string &text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        status_pub_->publish(msg);
    }

    // This method is called when we receive feedback from Nav2 about the current navigation goal.
    // It logs the current pose of the robot and the distance remaining to the goal, throttled to once every 5 seconds to avoid spamming the log.
    void MissionManager::onNavigationFeedback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        const auto &waypoint = current_waypoints_[current_waypoint_index_];
        const auto &q = feedback->current_pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Nav2 edge %d waypoint %zu (%.2f, %.2f): current pose (%.2f, %.2f, %.2f), distance remaining %.2f m",
            current_edge_id_,
            current_waypoint_index_ + 1,
            waypoint.x,
            waypoint.y,
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            yaw * 180.0 / M_PI,
            feedback->distance_remaining);
    }

} // namespace trail_decision_making
