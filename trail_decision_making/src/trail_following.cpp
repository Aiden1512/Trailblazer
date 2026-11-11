// Driving a trail edge: sending waypoints / the recorded trail line to Nav2 and handling the results.
#include "trail_decision_making/mission_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace trail_decision_making
{

    using namespace std::placeholders;    // for using _1, _2, etc. in std::bind
    using namespace std::chrono_literals; // for using 1s, 500ms, etc. for durations

    // Follow the edge's waypoints from current_node_id_ to its other end, driving to the entry waypoint first if we are not there.
    void MissionManager::startEdgeFollow(const TrailEdge &edge, bool reverse)
    {
        auto waypoints = edge.waypoints; // copy: reversed and thinned below without touching the graph
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

        // Drop near-duplicate waypoints (closer than waypoint_spacing_ to the last one kept), keeping the first and last.
        // Recorded and hand-added waypoints are ~1 m apart and each one keeps Nav2's planner on the trail, so
        // only duplicates go: a wider spacing (it was 1.2 m) threw away real waypoints and let the planner take
        // its own way between the ones left, e.g. a U-turn round the tree at the start of trail 2.
        std::vector<Waypoint> thinned;
        thinned.push_back(waypoints.front());
        for (size_t index = 1; index < waypoints.size(); ++index)
        {
            const double dx = waypoints[index].x - thinned.back().x;
            const double dy = waypoints[index].y - thinned.back().y;
            if (std::hypot(dx, dy) >= waypoint_spacing_)
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
        transit_to_edge_start_ = false;
        try
        {
            const auto pose = tf_buffer_->lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            const double dx = thinned.front().x - pose.transform.translation.x;
            const double dy = thinned.front().y - pose.transform.translation.y;
            const double distance_to_entry = std::hypot(dx, dy);
            if (distance_to_entry < 0.5 && current_waypoints_.size() > 1)
            {
                first_waypoint = 1;
                RCLCPP_INFO(get_logger(),
                            "Skipping edge %d start waypoint; robot is %.2f m from it",
                            edge.id, distance_to_entry);
            }
            else if (distance_to_entry >= 0.5)
            {
                transit_to_edge_start_ = true;
                RCLCPP_INFO(get_logger(),
                            "Transiting %.2f m to edge %d entry waypoint (%.2f, %.2f)",
                            distance_to_entry, edge.id,
                            thinned.front().x, thinned.front().y);
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not check current pose before navigation: %s", ex.what());
        }

        goal_retry_count_ = 0;
        current_waypoint_index_ = first_waypoint;
        sendNextWaypoint();
    }

    // send the next waypoint in the current edge to Nav2 for navigation.
    // If we have reached the last waypoint, we call onEdgeFinished() to handle the end of the edge traversal.
    void MissionManager::sendNextWaypoint()
    {
        if (paused_) // every goal goes out through here, so nothing moves the robot until resumed
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Mission e-stopped: not sending goals");
            return;
        }
        if (obstacle_pause_active_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Navigation paused: obstacle is inside the stop range");
            return;
        }
        if (followTrailLine())
        {
            sendTrailLine();
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
            publishStatus("Stopped: Nav2 is not available");
            mission_active_ = false;
            return;
        }
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

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = global_frame_;
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = waypoint.x;
        goal.pose.pose.position.y = waypoint.y;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, target_yaw);
        goal.pose.pose.orientation = tf2::toMsg(orientation);

        if (driving_to_home_pose_)
        {
            RCLCPP_INFO(get_logger(), "Sending home goal (%.2f, %.2f)", waypoint.x, waypoint.y);
            publishStatus("Returning to the home position - " + progressText());
        }
        else if (backtracking_)
        {
            RCLCPP_INFO(get_logger(), "Backtracking to %s: waypoint %zu/%zu (%.2f, %.2f)",
                        junctionName(current_node_id_).c_str(), current_waypoint_index_ + 1,
                        current_waypoints_.size(), waypoint.x, waypoint.y);
            publishStatus((home_requested_ ? "Heading home: back to " : trailName(current_edge_id_) + " blocked: returning to ") +
                          junctionName(current_node_id_) + " (" + std::to_string(current_waypoint_index_ + 1) +
                          "/" + std::to_string(current_waypoints_.size()) + ")");
        }
        else if (transit_to_edge_start_ && current_waypoint_index_ == 0)
        {
            RCLCPP_INFO(get_logger(), "Sending transit goal to edge %d entry waypoint",
                        current_edge_id_);
            publishStatus("Driving to the start of " + trailName(current_edge_id_, false));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Sending edge %d waypoint %zu/%zu: (%.2f, %.2f)",
                        current_edge_id_, current_waypoint_index_ + 1,
                        current_waypoints_.size(), waypoint.x, waypoint.y);
            publishStatus(std::string(returning_home_ ? "Heading home - " : "") +
                          trailName(current_edge_id_) +
                          (checking_far_side_ ? " (checking the far side of the obstacle)"
                                              : inspecting_edge_ ? " (inspecting)" : "") + ": waypoint " +
                          std::to_string(current_waypoint_index_ + 1) + "/" +
                          std::to_string(current_waypoints_.size()));
        }

        // Send the goal to Nav2 and set up callbacks for feedback and result
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        const unsigned seq = ++goal_seq_;
        options.goal_response_callback = [this, seq](
                                             const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            if (seq != goal_seq_) // cancelled (e-stop, obstacle) or replaced before Nav2 accepted it
            {
                if (goal_handle)
                {
                    nav_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "Nav2 rejected the navigation goal");
                publishStatus("Stopped: Nav2 rejected the goal");
                mission_active_ = false;
                return;
            }
            current_goal_handle_ = goal_handle;
        };
        // send the goal to Nav2 asynchronously so we can continue processing other events while Nav2 is navigating
        options.feedback_callback = [this, seq](GoalHandleNavigateToPose::SharedPtr handle,
                                                const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            if (seq == goal_seq_)
            {
                onNavigationFeedback(handle, feedback);
            }
        };
        options.result_callback = [this, seq](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            if (seq != goal_seq_)
            {
                RCLCPP_DEBUG(get_logger(), "Ignoring the result of a superseded NavigateToPose goal");
                return;
            }
            onNavigationResult(result);
        };
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
            handleGoalFailure(result.code);
            return;
        }

        goal_retry_count_ = 0;
        failed_recoveries_ = 0;
        transit_to_edge_start_ = false;
        if (going_around_ && go_around_timer_)
        {
            go_around_timer_->cancel(); // got past the obstacle in time
        }
        going_around_ = false; // reached the waypoint we were going round an obstacle to
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

    // A Nav2 goal (NavigateToPose or FollowPath) didn't succeed: ignore cancels (we cancel goals ourselves and
    // whoever cancelled resends), retry aborts a few times, then treat the edge as impassable.
    void MissionManager::handleGoalFailure(rclcpp_action::ResultCode code)
    {
        {
            // We only cancel goals ourselves (obstacle pause), and whoever cancelled
            // is responsible for resending, so a cancelled result is never a failure.
            if (code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_INFO(get_logger(), "Nav2 goal cancelled");
                return;
            }
            // An abort is often transient (e.g. stale costmap cells around the robot
            // making the planner fail), so resend the same waypoint a few times
            // before giving up on the edge.
            if (code == rclcpp_action::ResultCode::ABORTED &&
                goal_retry_count_ < max_goal_retries_)
            {
                ++goal_retry_count_;
                RCLCPP_WARN(get_logger(),
                            "Nav2 aborted edge %d waypoint %zu; retrying in %.1f s (%d/%d)",
                            current_edge_id_, current_waypoint_index_ + 1,
                            retry_delay_sec_, goal_retry_count_, max_goal_retries_);
                publishStatus("Nav2 aborted; retrying waypoint (" + std::to_string(goal_retry_count_) +
                              "/" + std::to_string(max_goal_retries_) + ")");
                retry_timer_ = create_wall_timer(
                    std::chrono::duration<double>(retry_delay_sec_), [this]()
                    {
                        retry_timer_->cancel();
                        if (mission_active_ && !obstacle_pause_active_)
                        {
                            sendNextWaypoint();
                        } });
                return;
            }
            RCLCPP_WARN(get_logger(),
                        "Nav2 failed edge %d: action result code=%d",
                        current_edge_id_, static_cast<int>(code));
            handleNavigationFailure();
        }
    }

    // Trail edges (inspecting, passing through, backtracking) are driven by handing Nav2's controller (RPP) the
    // recorded trail line itself. The robot drove these lines when the trails were mapped, but low branches put
    // costmap obstacles within centimetres of them, so the global planner would route round (U-turns, swinging
    // onto other trails). The planner is still used to get onto the network, round obstacles, and to the home pose.
    bool MissionManager::followTrailLine() const
    {
        return follow_trail_line_ && current_edge_id_ >= 0 && !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0) && !going_around_;
    }

    // After a trail-line goal was interrupted (obstacle, abort), move current_waypoint_index_ on to the first
    // waypoint the robot hasn't passed yet: the end of the line segment it is closest to.
    void MissionManager::updateWaypointIndexFromPose()
    {
        const auto &w = current_waypoints_;
        if (w.size() < 2 || current_waypoint_index_ >= w.size())
        {
            return;
        }
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &)
        {
            return;
        }
        const double x = tf.transform.translation.x, y = tf.transform.translation.y;
        size_t best = current_waypoint_index_;
        double best_d = std::numeric_limits<double>::max();
        for (size_t i = current_waypoint_index_ > 0 ? current_waypoint_index_ - 1 : 0; i + 1 < w.size(); ++i)
        {
            const double dx = w[i + 1].x - w[i].x, dy = w[i + 1].y - w[i].y;
            const double len2 = dx * dx + dy * dy;
            const double t = len2 > 1e-9 ? std::clamp(((x - w[i].x) * dx + (y - w[i].y) * dy) / len2, 0.0, 1.0) : 0.0;
            const double d = std::hypot(x - w[i].x - t * dx, y - w[i].y - t * dy);
            if (d < best_d)
            {
                best_d = d;
                best = i + 1;
            }
        }
        if (best < w.size() && std::hypot(x - w[best].x, y - w[best].y) < 0.3)
        {
            ++best; // right on that waypoint: it counts as reached
        }
        current_waypoint_index_ = std::max(current_waypoint_index_, best);
    }

    // Going round an obstacle: the next waypoint is usually less than a metre ahead, short of the obstacle (which
    // is about blocked_marker_ahead_ in front when we stop), so Nav2 would count it reached at once and the trail
    // line would lead straight back into the obstacle. Aim for the first waypoint at least go_around_ahead_
    // further along the trail instead, so the planner has to take us past it.
    void MissionManager::skipWaypointsPastObstacle()
    {
        auto &w = current_waypoints_;
        if (driving_to_home_pose_ || (transit_to_edge_start_ && current_waypoint_index_ == 0) ||
            current_waypoint_index_ >= w.size())
        {
            return;
        }
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &)
        {
            return;
        }
        size_t i = current_waypoint_index_;
        double along = std::hypot(w[i].x - tf.transform.translation.x, w[i].y - tf.transform.translation.y);
        while (along < go_around_ahead_ && i + 1 < w.size())
        {
            along += std::hypot(w[i + 1].x - w[i].x, w[i + 1].y - w[i].y);
            ++i;
        }
        RCLCPP_INFO(get_logger(), "Going around: aiming for waypoint %zu/%zu, %.1f m along the trail",
                    i + 1, w.size(), along);
        current_waypoint_index_ = i;
    }

    // One FollowPath goal from the robot along all the remaining waypoints of the edge, densified to 0.1 m
    void MissionManager::sendTrailLine()
    {
        if (line_goal_sent_)
        {
            updateWaypointIndexFromPose();
        }
        if (current_waypoint_index_ >= current_waypoints_.size())
        {
            line_goal_sent_ = false;
            onEdgeFinished();
            return;
        }
        if (!follow_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(get_logger(), "Nav2 controller action server 'follow_path' is not available");
            publishStatus("Stopped: Nav2 is not available");
            mission_active_ = false;
            return;
        }
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get pose before following the trail: %s", ex.what());
            return;
        }

        FollowPath::Goal goal;
        goal.controller_id = "FollowPath";
        goal.path.header.frame_id = global_frame_;
        goal.path.header.stamp = now();
        auto add_pose = [&](double x, double y, double yaw)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = goal.path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
            goal.path.poses.push_back(pose);
        };
        double px = tf.transform.translation.x, py = tf.transform.translation.y;
        double yaw = 0.0;
        for (size_t i = current_waypoint_index_; i < current_waypoints_.size(); ++i)
        {
            const Waypoint &w = current_waypoints_[i];
            yaw = std::atan2(w.y - py, w.x - px);
            const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(w.x - px, w.y - py) / 0.1)));
            for (int s = 0; s < steps; ++s)
            {
                add_pose(px + (w.x - px) * s / steps, py + (w.y - py) * s / steps, yaw);
            }
            px = w.x;
            py = w.y;
        }
        add_pose(px, py, yaw);

        const size_t remaining = current_waypoints_.size() - current_waypoint_index_;
        if (backtracking_)
        {
            RCLCPP_INFO(get_logger(), "Backtracking to %s along the trail (%zu waypoints)",
                        junctionName(current_node_id_).c_str(), remaining);
            publishStatus((home_requested_ ? "Heading home: back to " : trailName(current_edge_id_) + " blocked: returning to ") +
                          junctionName(current_node_id_));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Following edge %d along the trail from waypoint %zu/%zu",
                        current_edge_id_, current_waypoint_index_ + 1, current_waypoints_.size());
            publishStatus(std::string(returning_home_ ? "Heading home - " : "") + trailName(current_edge_id_) +
                          (checking_far_side_ ? " (checking the far side of the obstacle)"
                                              : inspecting_edge_ ? " (inspecting)" : "") +
                          ": following the trail, " + std::to_string(remaining) + " waypoints to go");
        }

        rclcpp_action::Client<FollowPath>::SendGoalOptions options;
        const unsigned seq = ++goal_seq_;
        options.goal_response_callback = [this, seq](const GoalHandleFollowPath::SharedPtr &goal_handle)
        {
            if (seq != goal_seq_) // cancelled (e-stop, obstacle) or replaced before Nav2 accepted it
            {
                if (goal_handle)
                {
                    follow_client_->async_cancel_goal(goal_handle);
                }
                return;
            }
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "Nav2 rejected the trail line");
                publishStatus("Stopped: Nav2 rejected the goal");
                mission_active_ = false;
                return;
            }
            follow_goal_handle_ = goal_handle;
        };
        options.feedback_callback = [this, seq](GoalHandleFollowPath::SharedPtr,
                                                const std::shared_ptr<const FollowPath::Feedback> feedback)
        {
            if (seq != goal_seq_)
            {
                return;
            }
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Following edge %d: %.2f m to go along the trail, %.2f m/s",
                                 current_edge_id_, feedback->distance_to_goal, feedback->speed);
        };
        options.result_callback = [this, seq](const GoalHandleFollowPath::WrappedResult &result)
        {
            if (seq != goal_seq_)
            {
                RCLCPP_DEBUG(get_logger(), "Ignoring the result of a superseded FollowPath goal");
                return;
            }
            onTrailLineResult(result);
        };
        line_goal_sent_ = true;
        follow_client_->async_send_goal(goal, options);
    }

    void MissionManager::onTrailLineResult(const GoalHandleFollowPath::WrappedResult &result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            handleGoalFailure(result.code); // a retry resends the line from wherever the robot now is
            return;
        }
        // The goal checker passed at the end of the line: the whole edge has been driven
        line_goal_sent_ = false;
        goal_retry_count_ = 0;
        failed_recoveries_ = 0;
        transit_to_edge_start_ = false;
        current_waypoint_index_ = current_waypoints_.size();
        RCLCPP_INFO(get_logger(), "Nav2 completed edge %d along the trail", current_edge_id_);
        onEdgeFinished();
    }

    // The stored handles aren't cleared when a goal finishes, and once its result is in the action client
    // forgets it and cancelling throws UnknownGoalHandleError; that just means there is nothing left to cancel.
    void MissionManager::cancelGoals()
    {
        ++goal_seq_; // also covers a goal sent but not yet accepted: its response callback cancels it
        if (current_goal_handle_)
        {
            try
            {
                nav_client_->async_cancel_goal(current_goal_handle_);
            }
            catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
            {
                RCLCPP_DEBUG(get_logger(), "NavigateToPose goal already finished; nothing to cancel");
            }
            current_goal_handle_.reset();
        }
        if (follow_goal_handle_)
        {
            try
            {
                follow_client_->async_cancel_goal(follow_goal_handle_);
            }
            catch (const rclcpp_action::exceptions::UnknownGoalHandleError &)
            {
                RCLCPP_DEBUG(get_logger(), "FollowPath goal already finished; nothing to cancel");
            }
            follow_goal_handle_.reset();
        }
    }

    // This method is called when we receive feedback from Nav2 about the current navigation goal.
    // It logs the current pose of the robot and the distance remaining to the goal, throttled to once every 5 seconds to avoid spamming the log.
    void MissionManager::onNavigationFeedback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        const auto &waypoint = current_waypoints_[current_waypoint_index_];
        const auto &q = feedback->current_pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Nav2 edge %d waypoint %zu (%.2f, %.2f): current pose (%.2f, %.2f, %.2f), distance remaining %.2f m",
            current_edge_id_,
            current_waypoint_index_ + 1,
            waypoint.x,
            waypoint.y,
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            yaw * 180.0 / M_PI,
            feedback->distance_remaining);
    }

} // namespace trail_decision_making
