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
        auto_start_ = declare_parameter<bool>("auto_start", false);

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
        return_pass_pending_ = false;
        return_pass_active_ = false;

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

        // Aiden change - Remember the mission's original junction.
        mission_start_node_id_ = current_node_id_;
        // Clear blocked-trail records for each new mission.
        blocked_trails_.clear();
        blocked_edge_id_ = -1;
        backtrack_entry_node_id_ = -1;
         // Aiden change - Reset opposite-side inspection tracking.
        inspection_phase_ = InspectionPhase::NORMAL;
        active_blocked_trail_index_ = 0;
        opposite_entry_node_id_ = -1;

        RCLCPP_INFO(
            get_logger(),"Mission starting at junction node %d", mission_start_node_id_);

        mission_active_ = true;      // set the mission as active so we can start traversing edges
        chooseNextEdgeAndSendGoal(); // choose the next edge to traverse and send the first waypoint of that edge
    }

    void MissionManager::chooseNextEdgeAndSendGoal()
    {
        if (!mission_active_) // if the mission is not active, we don't do anything
        {
            return;
        }

        // Aiden change - Stop at the opposite junction after completing transit.
        if (inspection_phase_ == InspectionPhase::TRANSIT_TO_OPPOSITE_SIDE &&
            route_.empty())
        {
            if (current_node_id_ != opposite_entry_node_id_)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Transit route ended at junction %d, expected junction %d",
                    current_node_id_,
                    opposite_entry_node_id_);

                mission_active_ = false;
                return;
            }

            inspection_phase_ = InspectionPhase::INSPECTING_OPPOSITE_SIDE;

            RCLCPP_INFO(
                get_logger(),
                "Reached opposite junction %d for blocked edge %d",
                opposite_entry_node_id_,
                blocked_trails_[active_blocked_trail_index_].edge_id);

        //Aiden change
            current_edge_id_ = -1;
            current_waypoints_.clear();
            startOppositeSideApproach();
            return;
        }

        if (inspection_phase_ == InspectionPhase::RETURNING_HOME && route_.empty())
        {
            if (current_node_id_ != mission_start_node_id_)
            {
                RCLCPP_ERROR(get_logger(), "Home route ended at junction %d, expected %d",
                             current_node_id_, mission_start_node_id_);
                publishStatus("Stopped: return-home route incomplete");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Mission complete: returned to starting junction %d",
                            mission_start_node_id_);
                publishStatus("Mission complete: returned to starting junction");
            }
            mission_active_ = false;
            return;
        }

        if (route_.empty()) // if we have no more edges to traverse
        {
            // Plan a route to the next uninspected edge from the current node
            const auto plan = planner_.planToNextUninspected(current_node_id_);
            
            //Aiden edit - if statement connects normal exploration to opposite side inspection
            if (plan.empty())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Normal exploration complete. Checking blocked trails.");

                if (!blocked_trails_.empty())
                {
                    beginOppositeSideInspection();
                    return;
                }

                RCLCPP_INFO(
                    get_logger(),
                    "No blocked trails require opposite-side inspection.");

        //Aiden change
                beginReturnHome();
        //Change end
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
        return_pass_pending_ = false; // aiden tweaked this
        return_pass_active_ = false;
        startEdgeFollow(*edge, traversing_reverse_); // start traversing the edge, either in reverse or not, depending on the current node
    }
    // This method is called when we have finished traversing an edge, either because we reached the last waypoint or because we finished a return pass of an uninspected edge.
    // It updates the current node and edge state, and either starts a return pass or chooses the next edge to traverse.
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
        //Aiden added
        if (recovery_state_ != RecoveryState::NONE)
        {
            RCLCPP_WARN(
                get_logger(),
                "Normal waypoint navigation disabled during obstacle recovery");
            return;
        }
        if (obstacle_stop_active_)
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

        if (transit_to_edge_start_ && current_waypoint_index_ == 0)
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
            publishStatus("Edge " + std::to_string(current_edge_id_) +
                          (return_pass_active_ ? " (return pass)" : "") + ": waypoint " +
                          std::to_string(current_waypoint_index_ + 1) + "/" +
                          std::to_string(current_waypoints_.size()));
        }

        // Send the goal to Nav2 and set up callbacks for feedback and result
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        navigation_goal_pending_ = true;

        options.goal_response_callback = [this](
            const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            navigation_goal_pending_ = false;

            if (!goal_handle)
            {
                if (recovery_state_ == RecoveryState::CANCELLING)
                {
                    beginBacktracking();
                    return;
                }

                RCLCPP_ERROR(get_logger(), "Nav2 rejected the navigation goal");

                publishStatus("Stopped: Nav2 rejected the goal");
                mission_active_ = false;
                return;
            }

            current_goal_handle_ = goal_handle;

            // Cancel the goal if an obstacle was detected
            // while Nav2 was processing the request.
            if (recovery_state_ == RecoveryState::CANCELLING)
            {
                RCLCPP_WARN(get_logger(),"Obstacle recovery active; cancelling newly accepted goal");
                nav_client_->async_cancel_goal(goal_handle);
            }
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
        current_goal_handle_.reset();

        if (recovery_state_ == RecoveryState::CANCELLING)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_WARN(get_logger(),"Navigation goal succeeded during obstacle cancellation");
            }

            beginBacktracking();
            return;
        }

        if (recovery_state_ != RecoveryState::NONE)
        {
            return;
        }

        // Check if the navigation was successful
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            if (obstacle_stop_active_)
            {
                RCLCPP_WARN(get_logger(), "Nav2 goal cancelled while obstacle stop is active");
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
                        if (mission_active_ && !obstacle_stop_active_)
                        {
                            sendNextWaypoint();
                        } });
                return;
            }
            RCLCPP_ERROR(get_logger(),
                         "Nav2 failed edge %d: action result code=%d",
                         current_edge_id_, static_cast<int>(result.code));
            publishStatus("Stopped: Nav2 failed edge " + std::to_string(current_edge_id_));
            mission_active_ = false;
            return;
        }

        goal_retry_count_ = 0;
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
    // or because we finished a return pass of an uninspected edge. It updates the current node and edge state, and
    // either starts a return pass or chooses the next edge to traverse.
    void MissionManager::onEdgeFinished()
    {
        const TrailEdge *edge = graph_.getEdge(current_edge_id_);
        if (!edge)
        {
            RCLCPP_ERROR(get_logger(),  "Could not finish missing edge %d", current_edge_id_);
            mission_active_ = false;
            return;
        }

        // Aiden change - a blocked edge must NEVER be accepted as a completed crossing.
        if (inspection_phase_ == InspectionPhase::INSPECTING_OPPOSITE_SIDE)
        {
            RCLCPP_ERROR(get_logger(),
                         "Reached end of blocked edge %d without detecting obstacle; stopping for safety",
                         current_edge_id_);
            publishStatus("Stopped: blocked edge crossed without LiDAR detection");
            mission_active_ = false;
            return;
        }

        //Aiden end

        // A successful traversal completes the inspection.
        if (edge->status == EdgeStatus::UNINSPECTED)
        {
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::INSPECTED);
            RCLCPP_INFO(get_logger(), "Edge %d inspected", current_edge_id_);
            publishStatus("Edge " + std::to_string(current_edge_id_) + " inspected");
        }

        // Move our logical position to the destination junction.
        current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_);

        // No automatic return pass.
        return_pass_pending_ = false;
        return_pass_active_ = false;

        // Find the next uninspected trail, using inspected trails for transit when necessary.
        chooseNextEdgeAndSendGoal();
    }

    //Aiden added
    void MissionManager::beginBacktracking()
    {
        if (!mission_active_)
        {
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "Beginning backtrack on edge %d towards entry node %d",blocked_edge_id_,backtrack_entry_node_id_);
        backtrack_waypoints_.clear();

        // Only use waypoints that Husky has already reached.
        size_t completed_count = std::min(current_waypoint_index_, current_waypoints_.size());

        for (size_t i = completed_count; i > 0; --i)
        {
            Waypoint waypoint = current_waypoints_[i - 1];

            // Face towards the previous waypoint.
            waypoint.yaw = std::atan2(std::sin(waypoint.yaw + M_PI), std::cos(waypoint.yaw + M_PI));

            backtrack_waypoints_.push_back(waypoint);
        }

        // Ensure the return route finishes at the original entry waypoint.
        if (!current_waypoints_.empty())
        {
            const Waypoint &entry = current_waypoints_.front();

            if (backtrack_waypoints_.empty() ||
                std::hypot(backtrack_waypoints_.back().x - entry.x, backtrack_waypoints_.back().y - entry.y) > 0.05)
            {
                backtrack_waypoints_.push_back(entry);
            }
        }

        // Aiden change - Return to the actual entry junction after reversing through the completed trail waypoints.
        const auto &nodes = graph_.nodes();

        auto junction_it = std::find_if(nodes.begin(),nodes.end(),[this](const TrailNode &node)
            {
                return node.id == backtrack_entry_node_id_;
            });

        if (junction_it == nodes.end())
        {
            RCLCPP_ERROR(get_logger(),"Cannot backtrack: entry junction node %d not found",backtrack_entry_node_id_);

            mission_active_ = false;
            return;
        }

        const Waypoint &junction = junction_it->position;

        if (backtrack_waypoints_.empty() ||std::hypot(
                backtrack_waypoints_.back().x - junction.x,
                backtrack_waypoints_.back().y - junction.y) > 0.05)
        {
            backtrack_waypoints_.push_back(junction);
        }

        RCLCPP_INFO(get_logger(),"Final backtracking destination: junction %d (%.2f, %.2f)",backtrack_entry_node_id_,
            junction.x,
            junction.y);

        recovery_state_ = RecoveryState::BACKTRACKING;
        backtrack_waypoint_index_ = 0;

        RCLCPP_INFO(get_logger(),"Backtrack prepared with %zu waypoints",backtrack_waypoints_.size());

        publishStatus("Backtracking to entry junction on edge " +std::to_string(blocked_edge_id_));

        // Navigation will be dispatched by the dedicated recovery function.
        sendNextBacktrackWaypoint();
    }

    void MissionManager::sendNextBacktrackWaypoint()
    {
        if (!mission_active_ ||
            recovery_state_ != RecoveryState::BACKTRACKING)
        {
            return;
        }

        // During recovery, allow Nav2 to evaluate the return path
        // even if the original obstacle remains detected.

        if (backtrack_waypoint_index_ >= backtrack_waypoints_.size())
        {
            onBacktrackingFinished();
            return;
        }

        if (!nav_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 unavailable during backtracking");
            recovery_state_ = RecoveryState::WAITING_FOR_REROUTE;
            mission_active_ = false;
            return;
        }

        const auto &waypoint =
            backtrack_waypoints_[backtrack_waypoint_index_];
         geometry_msgs::msg::TransformStamped transform;

        try
        {
            transform = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(),"Cannot get robot pose during backtracking: %s",ex.what());
            mission_active_ = false;
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

        RCLCPP_INFO(get_logger(),"Backtracking waypoint %zu/%zu: (%.2f, %.2f)", 
            backtrack_waypoint_index_ + 1,
            backtrack_waypoints_.size(),
            waypoint.x,
            waypoint.y);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(),"Nav2 rejected backtracking goal");
                mission_active_ = false;
                return;
            }

            current_goal_handle_ = goal_handle;
            // Cancel if recovery is no longer active.
            // The original forward obstacle-stop signal does not
            // automatically cancel a backtracking goal.
            if (recovery_state_ != RecoveryState::BACKTRACKING)
            {
                nav_client_->async_cancel_goal(goal_handle);
            }
        };

        options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            current_goal_handle_.reset();

            if (recovery_state_ != RecoveryState::BACKTRACKING)
            {
                return;
            }

            if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_ERROR(get_logger(),"Backtracking navigation failed or was cancelled");

                mission_active_ = false;
                recovery_state_ = RecoveryState::WAITING_FOR_REROUTE;
                return;
            }

            ++backtrack_waypoint_index_;
            sendNextBacktrackWaypoint();
        };

        nav_client_->async_send_goal(goal, options);
    }


    // Aiden change - Mark the obstructed trail as blocked and prepare an alternative route after returning to the entry junction.
    void MissionManager::onBacktrackingFinished()
    {
        RCLCPP_INFO(
            get_logger(),
            "Backtracking complete. Returned to entry junction node %d",
            backtrack_entry_node_id_);

    //An opposite-side inspection is complete only after a successful backtrack.
        if (inspection_phase_ == InspectionPhase::RETURNING_FROM_OPPOSITE_SIDE)
        {
            if (active_blocked_trail_index_ >= blocked_trails_.size() ||
                blocked_trails_[active_blocked_trail_index_].edge_id != blocked_edge_id_)
            {
                RCLCPP_ERROR(get_logger(), "Opposite-side recovery record mismatch");
                mission_active_ = false;
                return;
            }
            blocked_trails_[active_blocked_trail_index_].opposite_side_inspected = true;
            current_node_id_ = backtrack_entry_node_id_;
            current_edge_id_ = -1;
            current_waypoints_.clear();
            current_waypoint_index_ = 0;
            route_.clear();
            recovery_state_ = RecoveryState::WAITING_FOR_REROUTE;
            RCLCPP_INFO(get_logger(), "Opposite side of blocked edge %d inspected; returned to junction %d",
                        blocked_edge_id_, current_node_id_);
            publishStatus("Opposite side of blocked edge " + std::to_string(blocked_edge_id_) + " inspected");
            if (!obstacle_stop_active_) continueAfterRecovery();
            return;
        }

        // A newly blocked transit edge invalidates the old opposite-side route.
        inspection_phase_ = InspectionPhase::NORMAL;

        // Mark the obstructed trail as blocked.
        graph_.setEdgeStatus(blocked_edge_id_, EdgeStatus::BLOCKED);

        //Save each blocked trail for later opposite-side inspection.
        const auto existing_blocked = std::find_if(
            blocked_trails_.begin(),
            blocked_trails_.end(),
            [this](const BlockedTrailRecord &record)
            {
                return record.edge_id == blocked_edge_id_;
            });

        if (existing_blocked == blocked_trails_.end())
        {
            blocked_trails_.push_back({
                blocked_edge_id_,
                backtrack_entry_node_id_,
                false,
                false
            });

            RCLCPP_INFO(
                get_logger(),
                "Saved blocked trail %d for later opposite-side inspection", blocked_edge_id_);
        }

        RCLCPP_INFO(get_logger(),"Total blocked trails recorded: %zu", blocked_trails_.size());

        RCLCPP_WARN(
            get_logger(), "Edge %d marked as BLOCKED", blocked_edge_id_);

        // Clear the previous route, which may contain the blocked edge.
        route_.clear();

        // Husky has returned to the junction where it entered the trail.
        current_node_id_ = backtrack_entry_node_id_;

        // Clear the previous navigation state.
        current_edge_id_ = -1;
        current_waypoints_.clear();
        current_waypoint_index_ = 0;
        current_goal_handle_.reset();

        recovery_state_ = RecoveryState::WAITING_FOR_REROUTE;

        publishStatus(
            "Edge " + std::to_string(blocked_edge_id_) +
            " blocked; preparing alternative route");

        // Select an alternative route using the existing route planner.
        const auto alternative_route =
            planner_.planToNextUninspected(current_node_id_);

        if (alternative_route.empty())
        {
            RCLCPP_INFO(
                get_logger(),
                "No more reachable uninspected trails. Preparing opposite-side inspection.");

            if (obstacle_stop_active_)
            {
                RCLCPP_INFO(get_logger(), "Waiting for obstacle stop zone to clear before continuing");
                return;
            }
            continueAfterRecovery();
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Alternative route found with %zu edge(s). Next edge: %d",
            alternative_route.size(),
            alternative_route.front());

        route_.assign(
            alternative_route.begin(),
            alternative_route.end());

        publishStatus(
            "Alternative route found; next edge " +
            std::to_string(route_.front()));

            //Resume automatically when the stop zone is clear.
        if (!obstacle_stop_active_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Obstacle stop zone clear; starting alternative route");

            recovery_state_ = RecoveryState::NONE;

            chooseNextEdgeAndSendGoal();
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Alternative route ready; waiting for obstacle stop to clear");

            publishStatus("Alternative route ready; waiting for clear stop zone");
        }
        // Keep recovery in WAITING_FOR_REROUTE until we verify
        // that the new route can be started safely.
    }

    void MissionManager::beginOppositeSideInspection()
    {
        RCLCPP_INFO(
            get_logger(),
            "Searching for blocked trails requiring opposite-side inspection");

        for (size_t i = 0; i < blocked_trails_.size(); ++i)
        {
            auto &record = blocked_trails_[i];

            if (record.opposite_side_inspected ||
                record.opposite_side_unreachable)
            {
                continue;
            }

            const TrailEdge *edge = graph_.getEdge(record.edge_id);

            if (!edge)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Blocked edge %d not found",
                    record.edge_id);

                record.opposite_side_unreachable = true;
                continue;
            }

            // Identify the opposite junction without hardcoded IDs.
            int opposite_node = -1;

            if (edge->from_node == record.first_entry_node)
            {
                opposite_node = edge->to_node;
            }
            else if (edge->to_node == record.first_entry_node)
            {
                opposite_node = edge->from_node;
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Invalid entry node for blocked edge %d",
                    record.edge_id);

                record.opposite_side_unreachable = true;
                continue;
            }

            // Find a route that does not cross blocked edges.
            const auto route_to_opposite =
                planner_.planToNode(current_node_id_, opposite_node);

            if (current_node_id_ != opposite_node &&
                route_to_opposite.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Opposite junction %d for edge %d is unreachable",
                    opposite_node,
                    record.edge_id);

                record.opposite_side_unreachable = true;
                continue;
            }

            active_blocked_trail_index_ = i;
            opposite_entry_node_id_ = opposite_node;

            RCLCPP_INFO(
                get_logger(),
                "Selected blocked edge %d; opposite junction %d; route contains %zu edges",
                record.edge_id,
                opposite_node,
                route_to_opposite.size());

                if (!route_to_opposite.empty())
                {
                    inspection_phase_ =
                        InspectionPhase::TRANSIT_TO_OPPOSITE_SIDE;

                    route_.assign(
                        route_to_opposite.begin(),
                        route_to_opposite.end());

                    RCLCPP_INFO(
                        get_logger(),
                        "Navigating to opposite junction %d using %zu edges",
                        opposite_node,
                        route_.size());

                    chooseNextEdgeAndSendGoal();
                }
                else
                {
                    inspection_phase_ =
                        InspectionPhase::INSPECTING_OPPOSITE_SIDE;

                    RCLCPP_INFO(
                        get_logger(),
                        "Reached opposite junction %d for blocked edge %d",
                        opposite_node,
                        record.edge_id);

                    current_edge_id_ = -1;
                    current_waypoints_.clear();
                    startOppositeSideApproach();
                }

            return;
        }

        RCLCPP_INFO(get_logger(), "No further reachable blocked trails require opposite-side inspection");
        beginReturnHome();
    }

    void MissionManager::startOppositeSideApproach()
    {
        if (!mission_active_ || active_blocked_trail_index_ >= blocked_trails_.size() ||
            current_node_id_ != opposite_entry_node_id_)
        {
            RCLCPP_ERROR(get_logger(), "Cannot start opposite-side approach: invalid junction or record");
            mission_active_ = false;
            return;
        }
        if (obstacle_stop_active_)
        {
            RCLCPP_WARN(get_logger(), "Opposite-side approach waiting for stop zone to clear");
            publishStatus("Waiting for obstacle stop zone to clear at opposite junction");
            return;
        }
        const int edge_id = blocked_trails_[active_blocked_trail_index_].edge_id;
        const TrailEdge *edge = graph_.getEdge(edge_id);
        if (!edge || edge->status != EdgeStatus::BLOCKED ||
            (edge->from_node != current_node_id_ && edge->to_node != current_node_id_))
        {
            RCLCPP_ERROR(get_logger(), "Invalid blocked edge %d for opposite-side approach", edge_id);
            mission_active_ = false;
            return;
        }
        inspection_phase_ = InspectionPhase::INSPECTING_OPPOSITE_SIDE;
        current_edge_id_ = edge_id;
        traversing_reverse_ = (edge->to_node == current_node_id_);
        RCLCPP_INFO(get_logger(), "Starting opposite-side inspection of blocked edge %d from junction %d",
                    edge_id, current_node_id_);
        publishStatus("Inspecting opposite side of blocked edge " + std::to_string(edge_id));
        startEdgeFollow(*edge, traversing_reverse_);
    }

    void MissionManager::beginReturnHome()
    {
        if (!mission_active_) return;
        inspection_phase_ = InspectionPhase::RETURNING_HOME;
        route_.clear();
        if (current_node_id_ == mission_start_node_id_)
        {
            RCLCPP_INFO(get_logger(), "Mission complete: already at starting junction %d", mission_start_node_id_);
            publishStatus("Mission complete: at starting junction");
            mission_active_ = false;
            return;
        }
        const auto home_route = planner_.planToNode(current_node_id_, mission_start_node_id_);
        if (home_route.empty())
        {
            RCLCPP_ERROR(get_logger(), "Starting junction %d unreachable from junction %d",
                         mission_start_node_id_, current_node_id_);
            publishStatus("Stopped: starting junction unreachable");
            mission_active_ = false;
            return;
        }
        route_.assign(home_route.begin(), home_route.end());
        RCLCPP_INFO(get_logger(), "Returning home to junction %d via %zu edges",
                    mission_start_node_id_, route_.size());
        publishStatus("Returning to starting junction");
        chooseNextEdgeAndSendGoal();
    }

    void MissionManager::continueAfterRecovery()
    {
        if (!mission_active_ || obstacle_stop_active_ ||
            recovery_state_ != RecoveryState::WAITING_FOR_REROUTE) return;
        recovery_state_ = RecoveryState::NONE;
        if (inspection_phase_ == InspectionPhase::RETURNING_FROM_OPPOSITE_SIDE)
        {
            beginOppositeSideInspection();
        }
        else if (!route_.empty())
        {
            chooseNextEdgeAndSendGoal();
        }
        else
        {
            inspection_phase_ = InspectionPhase::NORMAL;
            chooseNextEdgeAndSendGoal();
        }
    }

    //Aiden edit
    void MissionManager::obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == obstacle_stop_active_)
        {
            return;
        }

        obstacle_stop_active_ = msg->data;

        if (obstacle_stop_active_)
        {
            if (!mission_active_ ||
                recovery_state_ != RecoveryState::NONE)
            {
                return;
            }

            // Ignore a stop at a junction when no trail goal is active.
            if (current_edge_id_ < 0 ||
                (!current_goal_handle_ && !navigation_goal_pending_) ||
                inspection_phase_ == InspectionPhase::RETURNING_FROM_OPPOSITE_SIDE)
            {
                return;
            }
            blocked_edge_id_ = current_edge_id_;
            backtrack_entry_node_id_ = current_node_id_;
            
            if (inspection_phase_ == InspectionPhase::INSPECTING_OPPOSITE_SIDE)
                inspection_phase_ = InspectionPhase::RETURNING_FROM_OPPOSITE_SIDE;
            recovery_state_ = RecoveryState::CANCELLING;

            if (retry_timer_)
            {
                retry_timer_->cancel();
            }

            RCLCPP_WARN(
                get_logger(),"Obstacle detected on edge %d; stopping inspection", blocked_edge_id_);

            publishStatus(
                "Obstacle detected on edge " + std::to_string(blocked_edge_id_));

            if (current_goal_handle_)
            {
                nav_client_->async_cancel_goal(current_goal_handle_);
            }
            else if (!navigation_goal_pending_)
            {
                beginBacktracking();
            }
            // Otherwise wait for the pending goal response.
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Obstacle stop signal cleared");

            if (recovery_state_ == RecoveryState::BACKTRACKING && mission_active_ &&!current_goal_handle_)
            {
                sendNextBacktrackWaypoint();
            }

            // Aiden change - Start the prepared alternative route once the stop zone becomes clear.
            else if (recovery_state_ == RecoveryState::WAITING_FOR_REROUTE && mission_active_)
            {
                RCLCPP_INFO(get_logger(), "Stop zone cleared; continuing mission");
                continueAfterRecovery();
            }
            else if (recovery_state_ == RecoveryState::NONE && mission_active_ &&
                     inspection_phase_ == InspectionPhase::INSPECTING_OPPOSITE_SIDE &&
                     !current_goal_handle_ && !navigation_goal_pending_ && current_waypoints_.empty())
            {
                startOppositeSideApproach();
            }
        }
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
