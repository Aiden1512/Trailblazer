#include "trail_decision_making/mission_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

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
        obstacle_wait_sec_ = declare_parameter<double>("obstacle_wait_sec", 4.0);
        auto_start_ = declare_parameter<bool>("auto_start", false);
        home_x_ = declare_parameter<double>("home_x", 6.5);
        home_y_ = declare_parameter<double>("home_y", 4.0);
        // true = mark the trail blocked and go back to the previous junction; false = let Nav2 plan around.
        // Read each time an obstacle is handled, so it can be changed live with `ros2 param set`.
        declare_parameter<bool>("backtrack_on_obstacle", true);

        // Initialize TF buffer and listener, and the navigation action client
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, nav_action_name_);

        obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/husky1/obstacle_stop", 10,
            std::bind(&MissionManager::obstacleStopCallback, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to /husky1/obstacle_stop");

        // Latched so the GUI gets the current status even if it starts after us
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/husky1/mission_status", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        progress_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/husky1/mission_progress", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        // Published continuously (water_guard ignores it once it goes stale), so no state change is missed
        follow_trail_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/follow_trail", 10);
        follow_trail_timer_ = create_wall_timer(200ms, [this]()
                                                {
            std_msgs::msg::Bool msg;
            msg.data = followingTrail();
            follow_trail_pub_->publish(msg); });
        // The mission is held until the GUI's start button calls this service (unless auto_start is set)
        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "/husky1/start_mission",
            std::bind(&MissionManager::onStartRequest, this, _1, _2));

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
        inspecting_edge_ = false;
        backtracking_ = false;
        returning_home_ = false;
        driving_to_home_pose_ = false;
        failed_recoveries_ = 0;

        RCLCPP_INFO(get_logger(), "Start command received");
        publishStatus("Start command received");
        start_timer_ = create_wall_timer(1s, [this]()
                                         { startMission(); });
        response->success = true;
        response->message = "Mission starting (" + std::to_string(graph_.edges().size()) + " trail edges)";
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
        inspecting_edge_ = (edge->status == EdgeStatus::UNINSPECTED); // false for already-inspected edges we are only passing through
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

        // Thin the waypoints to avoid sending too many navigation goals to Nav2.
        // Keep the first and last waypoints and thin intermediate goals to 1.2 m spacing.
        // This is to avoid sending too many navigation goals to Nav2, which can cause it to fail or take a long time to plan.
        std::vector<Waypoint> thinned;
        thinned.push_back(waypoints.front());
        for (size_t index = 1; index < waypoints.size(); ++index)
        {
            const double dx = waypoints[index].x - thinned.back().x;
            const double dy = waypoints[index].y - thinned.back().y;
            if (std::hypot(dx, dy) >= 1.2)
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
        if (obstacle_pause_active_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Navigation paused: obstacle is inside the stop range");
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
            publishStatus("Edge " + std::to_string(current_edge_id_) + " blocked: returning to " +
                          junctionName(current_node_id_) + " (" + std::to_string(current_waypoint_index_ + 1) +
                          "/" + std::to_string(current_waypoints_.size()) + ")");
        }
        else if (transit_to_edge_start_ && current_waypoint_index_ == 0)
        {
            RCLCPP_INFO(get_logger(), "Sending transit goal to edge %d entry waypoint",
                        current_edge_id_);
            publishStatus("Driving to the start of edge " + std::to_string(current_edge_id_));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Sending edge %d waypoint %zu/%zu: (%.2f, %.2f)",
                        current_edge_id_, current_waypoint_index_ + 1,
                        current_waypoints_.size(), waypoint.x, waypoint.y);
            publishStatus(std::string(returning_home_ ? "Heading home - " : "") + "Edge " +
                          std::to_string(current_edge_id_) +
                          (inspecting_edge_ ? " (inspecting)" : "") + ": waypoint " +
                          std::to_string(current_waypoint_index_ + 1) + "/" +
                          std::to_string(current_waypoints_.size()));
        }

        // Send the goal to Nav2 and set up callbacks for feedback and result
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback = [this](
                                             const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
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
        options.feedback_callback = std::bind(&MissionManager::onNavigationFeedback, this, _1, _2);
        options.result_callback = std::bind(&MissionManager::onNavigationResult, this, _1);
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
            // We only cancel goals ourselves (obstacle pause), and whoever cancelled
            // is responsible for resending, so a cancelled result is never a failure.
            if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_INFO(get_logger(), "Nav2 goal cancelled");
                return;
            }
            // An abort is often transient (e.g. stale costmap cells around the robot
            // making the planner fail), so resend the same waypoint a few times
            // before giving up on the edge.
            if (result.code == rclcpp_action::ResultCode::ABORTED &&
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
                        current_edge_id_, static_cast<int>(result.code));
            handleNavigationFailure();
            return;
        }

        goal_retry_count_ = 0;
        failed_recoveries_ = 0;
        transit_to_edge_start_ = false;
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
            publishProgress();
            publishStatus("Edge " + std::to_string(current_edge_id_) + " inspected - " + progressText());
            inspecting_edge_ = false;
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
            if (obstacle_avoidance_active_ || !mission_active_)
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
            if (current_goal_handle_)
            {
                nav_client_->async_cancel_goal(current_goal_handle_);
                current_goal_handle_.reset();
            }
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
               current_edge_id_ >= 0 &&
               !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0) &&
               !obstacle_pause_active_ &&
               !obstacle_avoidance_active_;
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
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::BLOCKED);
            publishProgress();
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
        graph_.setEdgeStatus(current_edge_id_, EdgeStatus::BLOCKED);
        route_.clear(); // the old route may go through the blocked edge
        inspecting_edge_ = false;
        RCLCPP_WARN(get_logger(), "%s edge %d; marked blocked, returning to %s",
                    reason.c_str(), current_edge_id_, junctionName(current_node_id_).c_str());
        publishProgress();
        publishStatus("Edge " + std::to_string(current_edge_id_) + " blocked by an obstacle - " + progressText());

        // Waypoints before current_waypoint_index_ have been reached; drive them in reverse, ending at the junction
        const size_t reached = std::max<size_t>(current_waypoint_index_, 1);
        std::vector<Waypoint> back(current_waypoints_.begin(), current_waypoints_.begin() + reached);
        std::reverse(back.begin(), back.end());
        current_waypoints_ = back;
        current_waypoint_index_ = 0;
        transit_to_edge_start_ = false;
        goal_retry_count_ = 0;
        backtracking_ = true;
        obstacle_avoidance_active_ = true; // the obstacle stays in the stop range while we turn away from it
        sendNextWaypoint();
    }

    // No reachable uninspected trail is left, so follow the trails back to the junction nearest home
    // (see chooseNextEdgeAndSendGoal), then sendHomeGoal() drives the last bit to the home position.
    void MissionManager::startReturnHome()
    {
        returning_home_ = true;
        const size_t unreachable = graph_.countEdges(EdgeStatus::UNINSPECTED);
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
        const size_t unreachable = graph_.countEdges(EdgeStatus::UNINSPECTED);
        RCLCPP_INFO(get_logger(), "Mission complete: %s, %zu blocked, %zu unreachable; robot is home",
                    progressText().c_str(), blocked, unreachable);
        publishProgress();
        publishStatus("Mission complete - " + progressText() + " (" +
                      std::to_string(graph_.countEdges(EdgeStatus::INSPECTED)) + " inspected, " +
                      std::to_string(blocked) + " blocked, " + std::to_string(unreachable) +
                      " unreachable). Robot is home");
    }

    // Completion = trails that have been resolved (inspected, or found blocked) out of all trails
    void MissionManager::publishProgress()
    {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(graph_.explorationPercentage() * 100.0);
        progress_pub_->publish(msg);
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
