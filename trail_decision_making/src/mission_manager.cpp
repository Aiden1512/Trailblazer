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
        waypoints_file_ = declare_parameter<std::string>("waypoints_file", "");
        global_frame_ = declare_parameter<std::string>("global_frame", "map");
        const auto nav_action =
            declare_parameter<std::string>("nav_action_name", "navigate_through_poses");

        nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, nav_action);

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

        // TODO: replace (0,0) with the robot's real pose from TF (map -> base_link)
        current_node_id_ = graph_.findNearestNode(0.0, 0.0);
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
        auto waypoints = edge.waypoints;
        if (reverse)
        {
            std::reverse(waypoints.begin(), waypoints.end());
        }

        NavigateThroughPoses::Goal goal;
        for (const auto &wp : waypoints)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = global_frame_;
            ps.header.stamp = now();
            ps.pose.position.x = wp.x;
            ps.pose.position.y = wp.y;
            const double yaw = reverse ? wp.yaw + M_PI : wp.yaw;
            ps.pose.orientation.z = std::sin(yaw / 2.0);
            ps.pose.orientation.w = std::cos(yaw / 2.0);
            goal.poses.push_back(ps);
        }

        rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;
        opts.goal_response_callback = std::bind(&MissionManager::goalResponseCallback, this, _1);
        opts.feedback_callback = std::bind(&MissionManager::feedbackCallback, this, _1, _2);
        opts.result_callback = std::bind(&MissionManager::resultCallback, this, _1);

        RCLCPP_INFO(get_logger(), "Driving edge %d ('%s')%s",
                    edge.id, edge.trail_name.c_str(), reverse ? " [reverse]" : "");
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
            RCLCPP_WARN(get_logger(), "Edge %d aborted, marking BLOCKED", current_edge_id_);
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::BLOCKED);
            route_.clear(); // replan from the current node
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