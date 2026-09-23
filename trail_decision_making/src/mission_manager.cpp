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
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");

        max_turn_rate_ = declare_parameter<double>("max_turn_rate", 1.0);
        max_speed_ = declare_parameter<double>("max_speed", 0.5);
        angle_tolerance_ = declare_parameter<double>("angle_tolerance", 0.1);
        goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.1);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/husky1/cmd_vel", 10); // cmd_vel not cmd_vel_nav

        start_timer_ = create_wall_timer(1s, [this]()
                                         {
        start_timer_->cancel();
        startMission(); });
    }

    void MissionManager::loadGraph()
    {
        RCLCPP_INFO(get_logger(), "Loading trail graph from '%s'", waypoints_file_.c_str());
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
        RCLCPP_INFO(get_logger(), "Starting mission with frames global='%s', robot='%s'",
                    global_frame_.c_str(), robot_frame_.c_str());
        loadGraph();
        if (graph_.edges().empty())
        {
            RCLCPP_ERROR(get_logger(), "Mission cannot start: graph contains no edges");
            return;
        }

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
            RCLCPP_INFO(get_logger(), "Planner returned %zu edge(s) from node %d",
                        plan.size(), current_node_id_);
            if (plan.empty())
            {
                RCLCPP_INFO(get_logger(), "Mission complete (%.0f%% explored)",
                            graph_.explorationPercentage() * 100.0);
                mission_active_ = false;
                return;
            }
            route_.assign(plan.begin(), plan.end());
            RCLCPP_INFO(get_logger(), "Route queue now contains %zu edge(s)", route_.size());
        }

        current_edge_id_ = route_.front();
        route_.pop_front();
        RCLCPP_INFO(get_logger(), "Selecting edge %d; %zu edge(s) remain in route queue",
                    current_edge_id_, route_.size());

        const TrailEdge *edge = graph_.getEdge(current_edge_id_);
        if (!edge)
        {
            RCLCPP_ERROR(get_logger(), "Could not find selected edge %d in graph", current_edge_id_);
            return;
        }
        traversing_reverse_ = (edge->to_node == current_node_id_);
        RCLCPP_INFO(get_logger(), "Edge %d connects node %d -> %d; current node=%d; reverse=%s",
                    edge->id, edge->from_node, edge->to_node, current_node_id_,
                    traversing_reverse_ ? "true" : "false");
        startEdgeFollow(*edge, traversing_reverse_);
    }

    void MissionManager::startEdgeFollow(const TrailEdge &edge, bool reverse)
    {
        auto wps = edge.waypoints;
        if (wps.empty())
        {
            RCLCPP_ERROR(get_logger(), "Edge %d has no waypoints; cannot start following it", edge.id);
            return;
        }
        if (reverse)
        {
            std::reverse(wps.begin(), wps.end());
        }

        std::vector<Waypoint> thinned;
        thinned.push_back(wps.front());
        for (size_t i = 1; i < wps.size(); ++i)
        {
            const double dx = wps[i].x - thinned.back().x;
            const double dy = wps[i].y - thinned.back().y;
            if (std::hypot(dx, dy) >= 0.6)
            {
                thinned.push_back(wps[i]);
            }
        }
        if (std::hypot(wps.back().x - thinned.back().x, wps.back().y - thinned.back().y) > 0.05)
        {
            thinned.push_back(wps.back());
        }

        current_path_ = thinned;
        current_wp_index_ = 1; // skip index 0, same as your recent change — it's the start point
        drive_state_ = DriveState::TURNING;

        RCLCPP_INFO(get_logger(), "Following edge %d ('%s') with %zu waypoints%s",
                    edge.id, edge.trail_name.c_str(), current_path_.size(), reverse ? " [reverse]" : "");
        for (size_t i = 0; i < current_path_.size(); ++i)
        {
            RCLCPP_INFO(get_logger(), "  waypoint[%zu]=(%.3f, %.3f)%s",
                        i, current_path_[i].x, current_path_[i].y,
                        i == current_wp_index_ ? " [initial target]" : "");
        }

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MissionManager::controlLoopCallback, this));
    }

    void MissionManager::controlLoopCallback()
    {
        if (current_path_.empty() || current_wp_index_ >= current_path_.size())
        {
            RCLCPP_ERROR(get_logger(), "Control loop has invalid path state: path_size=%zu wp_index=%zu",
                         current_path_.size(), current_wp_index_);
            return;
        }

        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "TF lookup failed in control loop: %s", ex.what());
            return;
        }

        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const auto &q = tf.transform.rotation;
        const double yaw = 2.0 * std::atan2(q.z, q.w);

        const auto &target = current_path_[current_wp_index_];
        const double dx = target.x - x;
        const double dy = target.y - y;
        const double dist = std::hypot(dx, dy);
        const double goal_angle = std::atan2(dy, dx);

        double angle_diff = goal_angle - yaw;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

        RCLCPP_INFO(get_logger(),
                    "control: edge=%d wp=%zu/%zu state=%d pos=(%.2f,%.2f) yaw=%.2f "
                    "target=(%.2f,%.2f) dist=%.2f goal_angle=%.2f angle_diff=%.2f "
                    "limits(speed=%.2f turn=%.2f tolerances(dist=%.2f angle=%.2f)",
                    current_edge_id_, current_wp_index_, current_path_.size() - 1,
                    static_cast<int>(drive_state_), x, y, yaw, target.x, target.y,
                    dist, goal_angle, angle_diff, max_speed_, max_turn_rate_,
                    goal_tolerance_, angle_tolerance_);

        geometry_msgs::msg::Twist cmd;
        switch (drive_state_)
        {
        case DriveState::IDLE:
            RCLCPP_INFO(get_logger(), "State IDLE -> TURNING");
            drive_state_ = DriveState::TURNING;
            break;
        case DriveState::TURNING:
            if (std::fabs(angle_diff) < angle_tolerance_)
            {
                RCLCPP_INFO(get_logger(), "Heading is within tolerance; state TURNING -> MOVING");
                drive_state_ = DriveState::MOVING;
            }
            else
            {
                const double min_turn = max_turn_rate_ * 0.5; // floor to overcome stiction
                double turn = std::clamp(angle_diff * 2.0, -max_turn_rate_, max_turn_rate_);
                if (angle_diff > 0)
                {
                    turn = std::max(turn, min_turn);
                }
                else
                {
                    turn = std::min(turn, -min_turn);
                }
                cmd.angular.z = turn;
                RCLCPP_INFO(get_logger(), "Turning toward target: angular.z=%.3f", cmd.angular.z);
            }
            break;
        case DriveState::MOVING:
            if (dist < goal_tolerance_)
            {
                RCLCPP_INFO(get_logger(), "Target reached: dist %.3f < tolerance %.3f; state MOVING -> STOPPING",
                            dist, goal_tolerance_);
                drive_state_ = DriveState::STOPPING;
            }
            else if (std::fabs(angle_diff) > angle_tolerance_ * 3.0)
            {
                RCLCPP_INFO(get_logger(), "Heading error %.3f is too large; state MOVING -> TURNING",
                            std::fabs(angle_diff));
                drive_state_ = DriveState::TURNING;
            }
            else
            {
                const double min_speed = max_speed_ * 0.2; // small floor, forward stiction is usually less severe
                cmd.linear.x = std::max(std::min(dist * 0.5, max_speed_), min_speed);
                RCLCPP_INFO(get_logger(), "Driving toward target: linear.x=%.3f", cmd.linear.x);
            }
            break;
        case DriveState::STOPPING:
            if (current_wp_index_ + 1 < current_path_.size())
            {
                ++current_wp_index_;
                RCLCPP_INFO(get_logger(), "Advancing to waypoint %zu/%zu; state STOPPING -> TURNING",
                            current_wp_index_, current_path_.size() - 1);
                drive_state_ = DriveState::TURNING;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Final waypoint reached on edge %d", current_edge_id_);
                control_timer_->cancel();
                onEdgeFinished();
                return;
            }
            break;
        }

        RCLCPP_INFO(get_logger(), "Publishing cmd_vel: linear.x=%.3f angular.z=%.3f",
                    cmd.linear.x, cmd.angular.z);
        cmd_vel_pub_->publish(cmd);
    }

    void MissionManager::onEdgeFinished()
    {
        if (graph_.getEdgeStatus(current_edge_id_) == EdgeStatus::UNINSPECTED)
        {
            graph_.setEdgeStatus(current_edge_id_, EdgeStatus::INSPECTED);
        }
        current_node_id_ = planner_.otherEnd(current_edge_id_, current_node_id_);
        chooseNextEdgeAndSendGoal();
    }

} // namespace trail_decision_making