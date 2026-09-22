#ifndef TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_
#define TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_

#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "trail_decision_making/route_planner.h"
#include "trail_decision_making/trail_graph.h"

namespace trail_decision_making
{

    class MissionManager : public rclcpp::Node
    {
    public:
        explicit MissionManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        // NavigateThroughPoses lets Nav2 follow all recorded waypoints of an edge in one goal
        using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

        // Core objects (declaration order matters: planner_ holds a reference to graph_)
        TrailGraph graph_;
        RoutePlanner planner_{graph_};
        rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
        rclcpp::TimerBase::SharedPtr start_timer_;

        // Mission state
        int current_node_id_{-1};
        int current_edge_id_{-1};
        bool traversing_reverse_{false};
        bool mission_active_{false};
        std::deque<int> route_; // queued edge ids

        // Parameters
        std::string waypoints_file_;
        std::string global_frame_{"map"};

        // Mission flow
        void loadGraph();
        void startMission();
        void chooseNextEdgeAndSendGoal();
        void sendEdgeGoal(const TrailEdge &edge, bool reverse);

        // Action callbacks
        void goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle);
        void feedbackCallback(
            GoalHandleNav::SharedPtr,
            const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
        void resultCallback(const GoalHandleNav::WrappedResult &result);

        // Later: subscribe to obstacle / blockage topic
        // void blockageCallback(...);
    };

} // namespace trail_decision_making

#endif // TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_