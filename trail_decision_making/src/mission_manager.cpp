#include "trail_decision_making/mission_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace trail_decision_making
{

    using namespace std::placeholders;
    using namespace std::chrono_literals;

    MissionManager::MissionManager(const rclcpp::NodeOptions &options)
        : rclcpp::Node("mission_manager", options)
    {
        waypoints_file_ = declare_parameter<std::string>("waypoints_file", ""); // maybe change to const char * for default value
        global_frame_ = declare_parameter<std::string>("global_frame", "map");
        const auto nav_action =
            declare_parameter<std::string>("nav_action_name", "navigate_through_poses");

        nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, nav_action);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // One-shot timer so startup happens after the node is fully constructed
        start_timer_ = create_wall_timer(1s, [this]()
                                         {
        start_timer_->cancel();
        startMission(); });
    }

    void MissionManager::loadGraph()
    {
        if (!graph_.loadFromYaml(waypoints_file_))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load trail graph from '%s'", waypoints_file_.c_str());
            return;
        }
        RCLCPP_INFO(
            get_logger(), "Loaded graph: %zu nodes, %zu edges",
            graph_.nodes().size(), graph_.edges().size());
    }

    void MissionManager::startMission()
    {
        loadGraph();
        if (graph_.edges().empty())
        {
            return;
        }

        if (!nav_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
            return;
        }

        // --------- TF lookup (more robust) ---------
        // const std::string robot_frame = "husky1/base_link"; // try this first
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");

        // Wait up to 15 seconds for the transform to appear
        if (!tf_buffer_->canTransform(global_frame_, robot_frame_, tf2::TimePointZero, 15s))
        {
            RCLCPP_ERROR(get_logger(),
                         "Timed out waiting for TF: %s → %s",
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
        // -------------------------------------------

        const double robot_x = transform.transform.translation.x;
        const double robot_y = transform.transform.translation.y;

        current_node_id_ = graph_.findNearestNode(robot_x, robot_y);
        RCLCPP_INFO(get_logger(), "Starting from nearest node %d (robot at %.2f, %.2f)",
                    current_node_id_, robot_x, robot_y);

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
            return;
        }
        traversing_reverse_ = (edge->to_node == current_node_id_);
        sendEdgeGoal(*edge, traversing_reverse_);
    }

    void MissionManager::sendEdgeGoal(const TrailEdge &edge, bool reverse)
    {
        auto wps = edge.waypoints;
        if (reverse)
        {
            std::reverse(wps.begin(), wps.end());
        }

        // --- Thin the waypoints (keep points at least ~0.6 m apart) ---
        std::vector<Waypoint> thinned;
        thinned.push_back(wps.front());
        for (size_t i = 1; i < wps.size(); ++i)
        {
            const double dx = wps[i].x - thinned.back().x;
            const double dy = wps[i].y - thinned.back().y;
            if (std::hypot(dx, dy) >= 0.6)
            { // tune this value
                thinned.push_back(wps[i]);
            }
        }
        // always keep the final point
        if (std::hypot(wps.back().x - thinned.back().x,
                       wps.back().y - thinned.back().y) > 0.05)
        {
            thinned.push_back(wps.back());
        }

        NavigateThroughPoses::Goal goal;
        for (size_t i = 1; i < thinned.size(); ++i) // TESTING: from 1 to skip the first point (robot's current position)
        {
            RCLCPP_INFO(get_logger(), "Waypoint %zu: x=%.2f y=%.2f", i, thinned[i].x, thinned[i].y);
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = global_frame_;
            ps.header.stamp = now();
            ps.pose.position.x = thinned[i].x;
            ps.pose.position.y = thinned[i].y;

            // Compute yaw from the direction of travel (much more stable)
            double yaw = 0.0;
            if (i + 1 < thinned.size())
            {
                yaw = std::atan2(thinned[i + 1].y - thinned[i].y,
                                 thinned[i + 1].x - thinned[i].x);
            }
            else if (i > 0)
            {
                yaw = std::atan2(thinned[i].y - thinned[i - 1].y,
                                 thinned[i].x - thinned[i - 1].x);
            }

            ps.pose.orientation.z = std::sin(yaw * 0.5);
            ps.pose.orientation.w = std::cos(yaw * 0.5);
            goal.poses.push_back(ps);
        }

        // --- Send the goal to Nav2 ---
        rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;
        opts.goal_response_callback =
            std::bind(&MissionManager::goalResponseCallback, this, std::placeholders::_1);
        opts.feedback_callback =
            std::bind(&MissionManager::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2); 
        opts.result_callback =
            std::bind(&MissionManager::resultCallback, this, std::placeholders::_1);

        // ... rest of the function (options + async_send_goal) stays the same
        RCLCPP_INFO(get_logger(), "Driving edge %d ('%s') with %zu poses%s",
                    edge.id, edge.trail_name.c_str(), goal.poses.size(),
                    reverse ? " [reverse]" : "");

        RCLCPP_INFO(get_logger(), "===== Sending %zu poses =====", goal.poses.size());
        for (size_t i = 0; i < goal.poses.size(); ++i)
        {
            const auto &p = goal.poses[i].pose.position;
            RCLCPP_INFO(get_logger(), "  pose[%zu]: x=%.2f  y=%.2f", i, p.x, p.y);
        }

        nav_client_->async_send_goal(goal, opts);
    }

    void MissionManager::goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal rejected by Nav2");
            // Treat like a blocked edge so the planner routes around it
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::BLOCKED);
            route_.clear();
            chooseNextEdgeAndSendGoal();
        }
    }

    void MissionManager::feedbackCallback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        RCLCPP_DEBUG(get_logger(), "Waypoints remaining: %d", feedback->number_of_poses_remaining);
    }

    void MissionManager::resultCallback(const GoalHandleNav::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (graph_.getEdgeStatus(current_edge_id_) == EdgeStatus::UNINSPECTED)
            {
                graph_.setEdgeStatus(current_edge_id_, EdgeStatus::INSPECTED);
            }
            current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_);
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Edge %d aborted – will retry / replan", current_edge_id_);
            // For now just replan. Only mark BLOCKED when your obstacle detector says so.
            route_.clear();
            break;

        case rclcpp_action::ResultCode::CANCELED:
            mission_active_ = false;
            return;

        default:
            return;
        }
        chooseNextEdgeAndSendGoal();
    }

} // namespace trail_decision_making