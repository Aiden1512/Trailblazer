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

        // Initialize TF buffer and listener, and the navigation action client
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, nav_action_name_);

        obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/husky1/obstacle_stop", 10,
            std::bind(&MissionManager::obstacleStopCallback, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to /husky1/obstacle_stop");

        start_timer_ = create_wall_timer(1s, [this]()
                                         { startMission(); });
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
            return;
        }
        // Check if TF is available before starting the mission
        if (!tf_buffer_->canTransform(global_frame_, robot_frame_, tf2::TimePointZero, 15s))
        {
            RCLCPP_WARN(get_logger(), "Waiting for TF: %s -> %s",
                        global_frame_.c_str(), robot_frame_.c_str());
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
        current_node_id_ = graph_.findNearestNode(
            transform.transform.translation.x,
            transform.transform.translation.y);
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
            // Plan a route to the next uninspected edge from the current node
            const auto plan = planner_.planToNextUninspected(current_node_id_);
            if (plan.empty()) // if there are no more uninspected edges, the mission is complete
            {
                RCLCPP_INFO(get_logger(), "Mission complete (%.0f%% explored)",
                            graph_.explorationPercentage() * 100.0);
                mission_active_ = false;
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
        return_pass_pending_ = (edge->status == EdgeStatus::UNINSPECTED); // if the edge is uninspected, we will do a return pass after the first traversal
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
        // We want to keep the first and last waypoints, and any waypoint that is at least 0.6 m away from the previous one.
        // This is to avoid sending too many navigation goals to Nav2, which can cause it to fail or take a long time to plan.
        std::vector<Waypoint> thinned;
        thinned.push_back(waypoints.front());
        for (size_t index = 1; index < waypoints.size(); ++index)
        {
            const double dx = waypoints[index].x - thinned.back().x;
            const double dy = waypoints[index].y - thinned.back().y;
            if (std::hypot(dx, dy) >= 0.6)
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
        try
        { // check if the robot is already close to the first waypoint, and if so, skip it to avoid sending a navigation goal that is too close to the current pose
            const auto pose = tf_buffer_->lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            const double dx = thinned.front().x - pose.transform.translation.x;
            const double dy = thinned.front().y - pose.transform.translation.y;
            if (std::hypot(dx, dy) < 0.5 && current_waypoints_.size() > 1)
            {
                first_waypoint = 1;
                RCLCPP_INFO(get_logger(),
                            "Skipping edge %d start waypoint; robot is %.2f m from it",
                            edge.id, std::hypot(dx, dy));
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not check current pose before navigation: %s", ex.what());
        }

        current_waypoint_index_ = first_waypoint; // set the current waypoint index to the first waypoint we want to navigate to
        sendNextWaypoint();                       // send the first waypoint to Nav2 to start navigating the edge
    }

    // send the next waypoint in the current edge to Nav2 for navigation.
    // If we have reached the last waypoint, we call onEdgeFinished() to handle the end of the edge traversal.
    void MissionManager::sendNextWaypoint()
    {
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
            mission_active_ = false;
            return;
        }
        // Align with the waypoint before driving when the heading error is significant.
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
        const auto &current_rotation = transform.transform.rotation;
        const double current_yaw = std::atan2(
            2.0 * (current_rotation.w * current_rotation.z +
                   current_rotation.x * current_rotation.y),
            1.0 - 2.0 * (current_rotation.y * current_rotation.y +
                         current_rotation.z * current_rotation.z));
        const double yaw_error = std::atan2(
            std::sin(target_yaw - current_yaw),
            std::cos(target_yaw - current_yaw));
        const bool start_rotation = !rotation_goal_active_ && std::fabs(yaw_error) > 0.35;

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = global_frame_;
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = start_rotation
                                        ? transform.transform.translation.x
                                        : waypoint.x;
        goal.pose.pose.position.y = start_rotation
                                        ? transform.transform.translation.y
                                        : waypoint.y;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, target_yaw);
        goal.pose.pose.orientation = tf2::toMsg(orientation);

        if (start_rotation)
        {
            rotation_goal_active_ = true;
            RCLCPP_INFO(get_logger(),
                        "Turning in place %.2f rad before edge %d waypoint %zu/%zu",
                        yaw_error, current_edge_id_, current_waypoint_index_ + 1,
                        current_waypoints_.size());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Sending edge %d waypoint %zu/%zu: (%.2f, %.2f)",
                        current_edge_id_, current_waypoint_index_ + 1,
                        current_waypoints_.size(), waypoint.x, waypoint.y);
        }

        // Send the goal to Nav2 and set up callbacks for feedback and result
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback = [this](
                                             const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "Nav2 rejected the navigation goal");
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
            if (obstacle_stop_active_)
            {
                RCLCPP_WARN(get_logger(), "Nav2 goal cancelled while obstacle stop is active");
                return;
            }
            RCLCPP_ERROR(get_logger(),
                         "Nav2 failed edge %d: action result code=%d",
                         current_edge_id_, static_cast<int>(result.code));
            mission_active_ = false;
            return;
        }

        if (rotation_goal_active_)
        {
            rotation_goal_active_ = false;
            sendNextWaypoint();
            return;
        }

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
        const TrailEdge *edge = graph_.getEdge(current_edge_id_); // get the edge again in case it was modified
        if (!edge)                                                // this should never happen, but just in case
        {
            RCLCPP_ERROR(get_logger(), "Could not finish missing edge %d", current_edge_id_);
            mission_active_ = false; // this is a fatal error, so we stop the mission
            return;
        }

        if (return_pass_pending_ && !return_pass_active_) // if we just finished the first pass of an uninspected edge, we need to do a return pass
        {
            current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_); // update the current node to the other end of the edge so we can traverse it in reverse
            return_pass_active_ = true;
            traversing_reverse_ = !traversing_reverse_;
            startEdgeFollow(*edge, traversing_reverse_); // start the return pass
            return;
        }

        if (return_pass_active_) // if we just finished the return pass of an uninspected edge, we mark it as inspected and clear the return pass flags
        {
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::INSPECTED); // mark the edge as inspected
            return_pass_pending_ = false;
            return_pass_active_ = false;
        }

        current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_); // update the current node to the other end of the edge so we can plan to the next uninspected edge
        chooseNextEdgeAndSendGoal();                                              // plan to the next uninspected edge and send the first waypoint of that edge
    }

    void MissionManager::obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == obstacle_stop_active_) // if the obstacle stop state has not changed, we don't need to do anything
        {
            return;
        }

        obstacle_stop_active_ = msg->data;
        if (obstacle_stop_active_)
        {
            RCLCPP_WARN(get_logger(), "Obstacle is inside the stop range; cancelling Nav2 goal");
            if (current_goal_handle_)
            {
                nav_client_->async_cancel_goal(current_goal_handle_);
                current_goal_handle_.reset();
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Obstacle stop cleared; navigation may resume");
            if (mission_active_)
            {
                sendNextWaypoint();
            }
        }
    }

    // This method is called when we receive feedback from Nav2 about the current navigation goal.
    // It logs the current pose of the robot and the distance remaining to the goal, throttled to once every 5 seconds to avoid spamming the log.
    void MissionManager::onNavigationFeedback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Nav2 edge %d waypoint %zu: current pose (%.2f, %.2f), distance remaining %.2f m",
            current_edge_id_, current_waypoint_index_ + 1,
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y, feedback->distance_remaining);
    }

} // namespace trail_decision_making
