#ifndef TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_
#define TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "trail_decision_making/route_planner.h"
#include "trail_decision_making/trail_graph.h"

namespace trail_decision_making
{

    class MissionManager : public rclcpp::Node
    {
    public:
        explicit MissionManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        enum class DriveState
        {
            IDLE,
            TURNING,
            MOVING,
            STOPPING
        };

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        DriveState drive_state_{DriveState::IDLE};
        std::vector<Waypoint> current_path_;
        size_t current_wp_index_{0};

        // Control parameters
        double max_turn_rate_= 1.0;
        double max_speed_ = 1.0;
        double angle_tolerance_ = 0.1;
        double goal_tolerance_ = 0.1;

        // Mission control methods
        void startEdgeFollow(const TrailEdge &edge, bool reverse);
        void controlLoopCallback();
        void onEdgeFinished();

        TrailGraph graph_;
        RoutePlanner planner_{graph_};
        rclcpp::TimerBase::SharedPtr start_timer_;

        // Current mission state
        int current_node_id_{-1};
        int current_edge_id_{-1};
        bool traversing_reverse_{false};
        bool return_pass_pending_{false};
        bool return_pass_active_{false};
        bool mission_active_{false};
        // a deque is used here because we will be popping from the front as we traverse edges
        std::deque<int> route_; // sequence of edge ids to traverse, in order, to reach the next uninspected edge

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string waypoints_file_;
        std::string global_frame_{"map"};
        std::string robot_frame_{"base_link"};

        void loadGraph();
        void startMission();
        void chooseNextEdgeAndSendGoal();
    };

} // namespace trail_decision_making

#endif // TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_