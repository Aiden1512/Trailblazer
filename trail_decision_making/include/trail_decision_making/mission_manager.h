#ifndef TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_
#define TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
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
        // TODO: Add an obstacle-status subscription here once the obstacle
        // message and topic are agreed with the perception/sensor component.

        // Mission control methods
        void startEdgeFollow(const TrailEdge &edge, bool reverse);
        void sendNextWaypoint();
        void onNavigationFeedback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void onNavigationResult(const GoalHandleNavigateToPose::WrappedResult &result);
        void onEdgeFinished();
        // TODO: Add an obstacle callback here. It should stop or cancel the
        // active Nav2 goal when the subscribed status reports an obstacle.

        TrailGraph graph_;
        RoutePlanner planner_{graph_};
        rclcpp::TimerBase::SharedPtr start_timer_;

        // Current mission state
        int current_node_id_ = -1;
        int current_edge_id_ = -1;
        bool traversing_reverse_ = false;
        bool return_pass_pending_ = false;
        bool return_pass_active_ = false;
        bool mission_active_ = false;
        // TODO: Add obstacle-stop state here if an obstacle stop must be
        // distinguished from normal completion or navigation failure.
        std::vector<Waypoint> current_waypoints_;
        size_t current_waypoint_index_ = 0;
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