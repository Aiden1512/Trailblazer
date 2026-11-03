#ifndef TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_
#define TRAIL_DECISION_MAKING__MISSION_MANAGER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
        using FollowPath = nav2_msgs::action::FollowPath;
        using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
        rclcpp_action::Client<FollowPath>::SharedPtr follow_client_; // RPP along the recorded trail line
        GoalHandleFollowPath::SharedPtr follow_goal_handle_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_stop_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_; // one-line progress for the GUI
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_; // mission completion, 0-100 %
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_pub_;     // trail network + statuses for the GUI
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follow_trail_pub_; // lets water_guard's trail-keeper steer
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blocked_points_pub_; // blocked spots for the GUI map
        rclcpp::TimerBase::SharedPtr follow_trail_timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;   // the GUI's "Start Autonomous Inspection" button
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;   // the GUI's "E-STOP" button
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;  // the GUI's "Resume" button
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;    // the GUI's "Return Home" button
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr obstacle_mode_srv_; // the GUI's obstacle mode toggle
        GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

        // Mission control methods
        void startEdgeFollow(const TrailEdge &edge, bool reverse);
        void sendNextWaypoint();
        void onNavigationFeedback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void onNavigationResult(const GoalHandleNavigateToPose::WrappedResult &result);
        void onTrailLineResult(const GoalHandleFollowPath::WrappedResult &result);
        void handleGoalFailure(rclcpp_action::ResultCode code);
        bool followTrailLine() const;
        void sendTrailLine();
        void updateWaypointIndexFromPose();
        void skipWaypointsPastObstacle();
        void cancelGoals();
        void onEdgeFinished();
        void obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void publishStatus(const std::string &text);
        void publishProgress();
        void publishGraph();
        std::string trailName(int edge_id, bool capital = true) const;
        std::string progressText() const;
        std::string junctionName(int node_id) const;
        bool onTrailEdge() const;
        bool followingTrail() const;
        bool canBacktrack() const;
        bool forcingBlockedEdge() const;
        void handleNavigationFailure();
        void replanFromNearestJunction();
        void startBacktrackToJunction(const std::string &reason);
        void driveBackToJunction();
        bool markCurrentEdgeBlocked();
        double flagInaccessibleSection();
        void recordBlockedPoint(bool avoided = false);
        void publishBlockedPoints();
        void startReturnHome();
        void sendHomeGoal();
        void finishMission();
        void onStartRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void onPauseRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void onResumeRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void onReturnHomeRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void onObstacleModeRequest(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        void haltRobot();
        void clearPause();
        void resumeDriving();
        double distanceHomeFrom(int node_id) const;

        TrailGraph graph_;
        RoutePlanner planner_{graph_};
        rclcpp::TimerBase::SharedPtr start_timer_;
        rclcpp::TimerBase::SharedPtr retry_timer_;
        rclcpp::TimerBase::SharedPtr obstacle_wait_timer_;

        // Current mission state
        int current_node_id_ = -1;
        int current_edge_id_ = -1;
        bool traversing_reverse_ = false;
        bool inspecting_edge_ = false; // current edge was uninspected, so finishing it marks it inspected
        bool checking_far_side_ = false; // inspecting a blocked edge from its other end, up to the obstacle

        struct BlockedPoint
        {
            int edge = -1;
            double x = 0.0, y = 0.0; // map frame
            int from_node = -1;      // junction the robot was coming from when it found it
            bool avoided = false;    // Nav2 went around it, so the trail is not blocked
            bool bounds_gap = false; // one of the two obstacles cutting off a stretch of trail between them
        };
        std::vector<BlockedPoint> blocked_points_;
        double blocked_marker_ahead_ = 1.2; // obstacle is about this far ahead of base_link when the robot stops
        // blocked from both ends: obstacles closer than this (m) along the trail are taken to be the same one
        double inaccessible_min_gap_ = 2.0;
        bool mission_active_ = false;
        bool obstacle_stop_active_ = false;      // latest value of the obstacle_stop topic
        bool obstacle_pause_active_ = false;     // stopped, waiting before trying to go around
        bool obstacle_avoidance_active_ = false; // Nav2 is replanning around the obstacle; stop topic ignored
        double obstacle_wait_sec_ = 4.0;
        bool backtracking_ = false;          // retracing the blocked edge back to the junction we came from
        bool returning_home_ = false;        // nothing left to inspect; following trails back home
        bool driving_to_home_pose_ = false;  // final leg from the home junction to (home_x_, home_y_)
        bool paused_ = false;                // e-stopped from the GUI: no goals are sent until resumed
        // Bumped for every goal sent and every cancel; callbacks of goals sent under an older number are ignored.
        // A goal Nav2 preempts with a newer one finishes ABORTED, which must not count as the new goal failing.
        unsigned goal_seq_ = 0;
        bool home_requested_ = false;        // the GUI's Return Home skipped the trails still to inspect
        double home_x_ = 6.5;
        double home_y_ = 4.0;
        int home_node_id_ = -1; // junction nearest the home position
        bool transit_to_edge_start_ = false;
        std::vector<Waypoint> current_waypoints_;
        size_t current_waypoint_index_ = 0;
        int goal_retry_count_ = 0; // consecutive aborts of the current waypoint
        int max_goal_retries_ = 3;
        int failed_recoveries_ = 0;     // Nav2 failures handled since a waypoint was last reached
        int max_failed_recoveries_ = 5; // more than this in a row and the mission stops
        double retry_delay_sec_ = 2.0;
        double waypoint_spacing_ = 0.4; // waypoints closer than this to the previous one are dropped as duplicates
        bool follow_trail_line_ = true; // drive trail edges along the recorded line (RPP only), not via the planner
        bool line_goal_sent_ = false;   // a FollowPath goal for the current waypoints is (or was) in progress
        bool going_around_ = false;     // Nav2's planner is taking us round an obstacle to the current waypoint
        double go_around_ahead_ = 2.5;  // going round, aim for the first waypoint at least this far (m) along the trail
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