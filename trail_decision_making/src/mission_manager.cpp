#include "trail_decision_making/mission_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace trail_decision_making
{

    using namespace std::placeholders;
    using namespace std::chrono_literals;

    MissionManager::MissionManager(const rclcpp::NodeOptions &options)
        : rclcpp::Node("mission_manager", options)
    {
        waypoints_file_ = declare_parameter<std::string>("waypoints_file", "");
        global_frame_ = declare_parameter<std::string>("global_frame", "map");
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
        nav_action_name_ = declare_parameter<std::string>(
            "nav_action_name", "navigate_to_pose");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, nav_action_name_);

        start_timer_ = create_wall_timer(1s, [this]()
                                         { startMission(); });
    }

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

    void MissionManager::startMission()
    {
        RCLCPP_INFO(get_logger(), "Starting mission with frames global='%s', robot='%s'",
                    global_frame_.c_str(), robot_frame_.c_str());
        loadGraph();
        if (graph_.edges().empty())
        {
            RCLCPP_ERROR(get_logger(), "Mission cannot start: graph contains no edges");
            return;
        }

        if (!tf_buffer_->canTransform(global_frame_, robot_frame_, tf2::TimePointZero, 15s))
        {
            RCLCPP_WARN(get_logger(), "Waiting for TF: %s -> %s",
                        global_frame_.c_str(), robot_frame_.c_str());
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(), "Could not get robot pose: %s", ex.what());
            return;
        }

        start_timer_->cancel();
        RCLCPP_INFO(get_logger(), "TF is available; starting mission");
        current_node_id_ = graph_.findNearestNode(
            transform.transform.translation.x,
            transform.transform.translation.y);
        mission_active_ = true;
        chooseNextEdgeAndSendGoal();
    }

    void MissionManager::chooseNextEdgeAndSendGoal()
    {
        if (!mission_active_)
        {
            return;
        }

        if (route_.empty())
        {
            const auto plan = planner_.planToNextUninspected(current_node_id_);
            if (plan.empty())
            {
                RCLCPP_INFO(get_logger(), "Mission complete (%.0f%% explored)",
                            graph_.explorationPercentage() * 100.0);
                mission_active_ = false;
                return;
            }
            route_.assign(plan.begin(), plan.end());
        }

        current_edge_id_ = route_.front();
        route_.pop_front();
        const TrailEdge *edge = graph_.getEdge(current_edge_id_);
        if (!edge)
        {
            RCLCPP_ERROR(get_logger(), "Could not find selected edge %d in graph", current_edge_id_);
            mission_active_ = false;
            return;
        }

        traversing_reverse_ = (edge->to_node == current_node_id_);
        return_pass_pending_ = (edge->status == EdgeStatus::UNINSPECTED);
        return_pass_active_ = false;
        startEdgeFollow(*edge, traversing_reverse_);
    }

    void MissionManager::startEdgeFollow(const TrailEdge &edge, bool reverse)
    {
        auto waypoints = edge.waypoints;
        if (waypoints.empty())
        {
            RCLCPP_ERROR(get_logger(), "Edge %d has no waypoints; cannot navigate", edge.id);
            mission_active_ = false;
            return;
        }
        if (reverse)
        {
            std::reverse(waypoints.begin(), waypoints.end());
            for (auto &waypoint : waypoints)
            {
                waypoint.yaw = std::atan2(
                    std::sin(waypoint.yaw + M_PI),
                    std::cos(waypoint.yaw + M_PI));
            }
        }

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
        if (std::hypot(waypoints.back().x - thinned.back().x,
                       waypoints.back().y - thinned.back().y) > 0.05)
        {
            thinned.push_back(waypoints.back());
        }

        current_waypoints_ = thinned;
        size_t first_waypoint = 0;
        try
        {
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

        current_waypoint_index_ = first_waypoint;
        sendNextWaypoint();
    }

    void MissionManager::sendNextWaypoint()
    {
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

        const auto &waypoint = current_waypoints_[current_waypoint_index_];
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = global_frame_;
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = waypoint.x;
        goal.pose.pose.position.y = waypoint.y;
        double target_yaw = waypoint.yaw;
        if (current_waypoint_index_ + 1 < current_waypoints_.size())
        {
            const auto &next_waypoint = current_waypoints_[current_waypoint_index_ + 1];
            target_yaw = std::atan2(
                next_waypoint.y - waypoint.y,
                next_waypoint.x - waypoint.x);
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, target_yaw);
        goal.pose.pose.orientation = tf2::toMsg(orientation);

        RCLCPP_INFO(get_logger(), "Sending edge %d waypoint %zu/%zu: (%.2f, %.2f)",
                    current_edge_id_, current_waypoint_index_ + 1,
                    current_waypoints_.size(), waypoint.x, waypoint.y);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback = [this](
                                             const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "Nav2 rejected the navigation goal");
                mission_active_ = false;
            }
        };
        options.feedback_callback = std::bind(&MissionManager::onNavigationFeedback, this, _1, _2);
        options.result_callback = std::bind(&MissionManager::onNavigationResult, this, _1);
        nav_client_->async_send_goal(goal, options);
    }

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

    void MissionManager::onNavigationResult(
        const GoalHandleNavigateToPose::WrappedResult &result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_ERROR(get_logger(),
                         "Nav2 failed edge %d: action result code=%d",
                         current_edge_id_, static_cast<int>(result.code));
            mission_active_ = false;
            return;
        }

        ++current_waypoint_index_;
        if (current_waypoint_index_ < current_waypoints_.size())
        {
            sendNextWaypoint();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Nav2 completed edge %d", current_edge_id_);
            onEdgeFinished();
        }
    }

    void MissionManager::onEdgeFinished()
    {
        const TrailEdge *edge = graph_.getEdge(current_edge_id_);
        if (!edge)
        {
            RCLCPP_ERROR(get_logger(), "Could not finish missing edge %d", current_edge_id_);
            mission_active_ = false;
            return;
        }

        if (return_pass_pending_ && !return_pass_active_)
        {
            current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_);
            return_pass_active_ = true;
            traversing_reverse_ = !traversing_reverse_;
            startEdgeFollow(*edge, traversing_reverse_);
            return;
        }

        if (return_pass_active_)
        {
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::INSPECTED);
            return_pass_pending_ = false;
            return_pass_active_ = false;
        }

        current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_);
        chooseNextEdgeAndSendGoal();
    }

} // namespace trail_decision_making
