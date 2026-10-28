#ifndef TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_
#define TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_stop_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_; // one-line progress for the GUI
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;   // the GUI's "Start Autonomous Inspection" button
        GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

        // Mission control methods
        void startEdgeFollow(const TrailEdge &edge, bool reverse);
        void sendNextWaypoint();
        void onNavigationFeedback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void onNavigationResult(const GoalHandleNavigateToPose::WrappedResult &result);
        void onEdgeFinished();
        void obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void publishStatus(const std::string &text);
        void onStartRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        TrailGraph graph_;
        RoutePlanner planner_{graph_};
        rclcpp::TimerBase::SharedPtr start_timer_;
        rclcpp::TimerBase::SharedPtr retry_timer_;

        // Current mission state
        int current_node_id_ = -1;
        int current_edge_id_ = -1;
        bool traversing_reverse_ = false;
        bool return_pass_pending_ = false;
        bool return_pass_active_ = false;
        bool mission_active_ = false;
        bool obstacle_stop_active_ = false;
        bool transit_to_edge_start_ = false;
        std::vector<Waypoint> current_waypoints_;
        size_t current_waypoint_index_ = 0;
        int goal_retry_count_ = 0; // consecutive aborts of the current waypoint
        int max_goal_retries_ = 3;
        double retry_delay_sec_ = 2.0;
        bool auto_start_ = false; // start as soon as TF is up instead of waiting for /husky1/start_mission
        // a deque is used here because we will be popping from the front as we traverse edges
        std::deque<int> route_; // sequence of edge ids to traverse, in order, to reach the next uninspected edge

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string waypoints_file_;
        std::string global_frame_{"map"};
        std::string robot_frame_{"base_link"};
        std::string nav_action_name_{"navigate_to_pose"};

        void loadGraph();
        void startMission();
        void chooseNextEdgeAndSendGoal();
    };

} // namespace trail_decision_making

#endif // TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_