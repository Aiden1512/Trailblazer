// Obstacles: stopping, going around, or marking the trail blocked and backtracking to replan.
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

    // When an obstacle enters the stop range we cancel the Nav2 goal so the robot stops, and wait
    // obstacle_wait_sec_ for the obstacle to move away. If it is still there, either:
    //  - backtrack_on_obstacle: mark the edge blocked, retrace it to the junction we came from and
    //    replan the rest of the inspection from there, or
    //  - otherwise: resend the current waypoint so Nav2 plans a path around it.
    // While Nav2 is going around (or we are backtracking away from the obstacle), the stop topic is
    // ignored until it clears, as the robot will pass close to the obstacle.
    void MissionManager::obstacleStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == obstacle_stop_active_) // if the obstacle stop state has not changed, we don't need to do anything
        {
            return;
        }

        obstacle_stop_active_ = msg->data;
        if (obstacle_stop_active_)
        {
            if (obstacle_avoidance_active_ || !mission_active_ || paused_)
            {
                return; // already going around an obstacle (let Nav2 handle it), or not driving at all
            }
            RCLCPP_WARN(get_logger(),
                        "Obstacle is inside the stop range; stopping for %.1f s before %s",
                        obstacle_wait_sec_, canBacktrack() ? "backtracking" : "going around");
            publishStatus("Obstacle ahead: stopped");
            obstacle_pause_active_ = true;
            if (retry_timer_)
            {
                retry_timer_->cancel();
            }
            cancelGoals();
            obstacle_wait_timer_ = create_wall_timer(
                std::chrono::duration<double>(obstacle_wait_sec_), [this]()
                {
                    obstacle_wait_timer_->cancel();
                    obstacle_pause_active_ = false;
                    if (!mission_active_)
                    {
                        return;
                    }
                    if (canBacktrack())
                    {
                        startBacktrackToJunction("Obstacle still blocking");
                        return;
                    }
                    RCLCPP_INFO(get_logger(), "Obstacle still present; asking Nav2 to plan around it");
                    if (line_goal_sent_)
                    {
                        updateWaypointIndexFromPose(); // go round to the next waypoint the robot hasn't passed
                        line_goal_sent_ = false;
                    }
                    recordBlockedPoint(true); // still facing it: mark it on the GUI map before turning away
                    if (!going_around_) // stopped again part way round: keep the first start and deadline
                    {
                        go_around_start_index_ = current_waypoint_index_;
                        startGoAroundTimer();
                    }
                    skipWaypointsPastObstacle();
                    going_around_ = true; // the planner, not the trail line, gets us past it
                    obstacle_avoidance_active_ = true;
                    goal_retry_count_ = 0;
                    sendNextWaypoint(); });
        }
        else
        {
            obstacle_avoidance_active_ = false;
            if (obstacle_pause_active_) // obstacle moved away during the wait; resume straight away
            {
                obstacle_wait_timer_->cancel();
                obstacle_pause_active_ = false;
                RCLCPP_INFO(get_logger(), "Obstacle stop cleared; resuming navigation");
                if (mission_active_)
                {
                    sendNextWaypoint();
                }
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Obstacle stop cleared");
            }
        }
    }

    // True while driving along a trail edge, i.e. there is an edge to block and a junction to go back to.
    // Not while driving to an edge's entry from off the trail network, on the final leg home, or while
    // already backtracking.
    bool MissionManager::onTrailEdge() const
    {
        return current_edge_id_ >= 0 &&
               !backtracking_ &&
               !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0);
    }

    // While true, water_guard's camera trail-keeper may steer the robot back onto the brown trail when Nav2
    // cuts a corner. Not while driving onto the network or to the home pose (both off the trail), nor while
    // stopped for or going around an obstacle (Nav2 has to leave the trail to get past it).
    bool MissionManager::followingTrail() const
    {
        return mission_active_ &&
               !paused_ &&
               current_edge_id_ >= 0 &&
               !driving_to_home_pose_ &&
               !(transit_to_edge_start_ && current_waypoint_index_ == 0) &&
               !obstacle_pause_active_ &&
               !obstacle_avoidance_active_ &&
               !going_around_;
    }

    // Obstacles are only backtracked from on a trail edge, and only in backtrack mode;
    // otherwise Nav2 goes around the obstacle instead.
    bool MissionManager::canBacktrack() const
    {
        return get_parameter("backtrack_on_obstacle").as_bool() && onTrailEdge() && !forcingBlockedEdge();
    }

    // On the way home through a trail already known to be blocked (because every route home is): backing off
    // it again would just loop, so Nav2 goes around the obstacle instead.
    bool MissionManager::forcingBlockedEdge() const
    {
        return returning_home_ && current_edge_id_ >= 0 && !backtracking_ &&
               graph_.getEdgeStatus(current_edge_id_) == EdgeStatus::BLOCKED;
    }

    // Going around an obstacle has go_around_timeout_sec_ to get past it (reach the waypoint beyond it); restarted
    // after an e-stop. 0 turns the limit off.
    void MissionManager::startGoAroundTimer()
    {
        if (go_around_timer_)
        {
            go_around_timer_->cancel();
        }
        if (go_around_timeout_sec_ <= 0.0)
        {
            return;
        }
        go_around_timer_ = create_wall_timer(
            std::chrono::duration<double>(go_around_timeout_sec_), [this]()
            {
                go_around_timer_->cancel();
                onGoAroundTimeout(); });
    }

    // Still not past the obstacle: give up going around, mark the trail blocked and backtrack to the junction
    // (as backtrack mode would have), reporting that the go-around was tried. Not on the only way home
    // (forcingBlockedEdge), where backtracking would loop, nor off the trail network, with no junction to go back to.
    void MissionManager::onGoAroundTimeout()
    {
        if (!mission_active_ || !going_around_ || paused_)
        {
            return;
        }
        if (!onTrailEdge() || forcingBlockedEdge())
        {
            RCLCPP_WARN(get_logger(), "Go-around not done after %.0f s, but there is nothing to backtrack to; "
                                      "still trying", go_around_timeout_sec_);
            return;
        }
        RCLCPP_WARN(get_logger(), "Could not get around the obstacle on edge %d within %.0f s; backtracking",
                    current_edge_id_, go_around_timeout_sec_);
        if (retry_timer_)
        {
            retry_timer_->cancel();
        }
        if (obstacle_wait_timer_)
        {
            obstacle_wait_timer_->cancel(); // stopped for the obstacle again part way round: don't wait any longer
        }
        obstacle_pause_active_ = false;
        cancelGoals();
        startBacktrackToJunction("Go-around timed out on",
                                 "Tried to go around the obstacle but couldn't within " +
                                     std::to_string(static_cast<int>(std::round(go_around_timeout_sec_))) +
                                     " s, backtracking: ");
    }

    // Nav2 gave up on a waypoint (after retries). Rather than ending the mission, treat the edge as
    // impassable and carry on inspecting whatever is still reachable, the long way round if need be.
    void MissionManager::handleNavigationFailure()
    {
        if (++failed_recoveries_ > max_failed_recoveries_) // stuck: every recovery since the last reached waypoint failed too
        {
            RCLCPP_ERROR(get_logger(), "Nav2 failed %d times in a row; stopping the mission", failed_recoveries_);
            publishStatus("Stopped: Nav2 keeps failing - " + progressText());
            mission_active_ = false;
            return;
        }
        if (driving_to_home_pose_)
        {
            RCLCPP_ERROR(get_logger(), "Nav2 could not reach the home position");
            publishStatus("Inspection finished - " + progressText() + ", but Nav2 could not reach the home position");
            mission_active_ = false;
            return;
        }
        if (forcingBlockedEdge()) // Nav2 couldn't get around the obstacle on the only way home either
        {
            RCLCPP_ERROR(get_logger(), "Nav2 cannot get past the obstacle on edge %d, the only trail home; stopping",
                         current_edge_id_);
            publishStatus("Stopped: every trail home is blocked - " + progressText());
            mission_active_ = false;
            return;
        }
        if (onTrailEdge()) // part way along the edge: block it and drive back to the junction we came from
        {
            startBacktrackToJunction("Nav2 cannot get along");
            return;
        }
        if (current_edge_id_ >= 0 && !backtracking_) // could not even reach the edge's entry from here, so don't try it again
        {
            markCurrentEdgeBlocked();
            RCLCPP_WARN(get_logger(), "Could not reach the start of edge %d; marked blocked", current_edge_id_);
        }
        // Could not reach the edge entry, or could not drive back to the junction: carry on from the nearest junction
        backtracking_ = false;
        replanFromNearestJunction();
    }

    void MissionManager::replanFromNearestJunction()
    {
        try
        {
            const auto pose = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            current_node_id_ = graph_.findNearestNode(pose.transform.translation.x, pose.transform.translation.y);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get robot pose to replan; using %s: %s",
                        junctionName(current_node_id_).c_str(), ex.what());
        }
        route_.clear();
        inspecting_edge_ = false;
        RCLCPP_INFO(get_logger(), "Replanning from %s", junctionName(current_node_id_).c_str());
        publishStatus("Replanning from " + junctionName(current_node_id_) + " - " + progressText());
        chooseNextEdgeAndSendGoal();
    }

    // The edge can't be driven (obstacle still there after the wait, or Nav2 gave up): mark it blocked, then
    // retrace the waypoints already driven back to the junction this traversal started from (current_node_id_).
    // When that is done onEdgeFinished() replans the rest of the inspection, which never uses blocked edges.
    void MissionManager::startBacktrackToJunction(const std::string &reason, const std::string &status_prefix)
    {
        if (going_around_)
        {
            // skipWaypointsPastObstacle() moved the index past the obstacle; we never got there, so retrace
            // only the waypoints reached before it
            current_waypoint_index_ = go_around_start_index_;
            line_goal_sent_ = false;
            if (go_around_timer_)
            {
                go_around_timer_->cancel();
            }
        }
        if (line_goal_sent_)
        {
            updateWaypointIndexFromPose(); // which waypoints were reached before the obstacle
            line_goal_sent_ = false;
        }
        going_around_ = false;
        const bool far_side_next = markCurrentEdgeBlocked();
        const double cut_off = checking_far_side_ ? flagInaccessibleSection() : 0.0;
        route_.clear(); // the old route may go through the blocked edge
        inspecting_edge_ = false;
        checking_far_side_ = false;
        RCLCPP_WARN(get_logger(), "%s edge %d; marked blocked, returning to %s%s",
                    reason.c_str(), current_edge_id_, junctionName(current_node_id_).c_str(),
                    far_side_next ? " (will check it from the other end later)" : "");
        publishStatus(status_prefix + trailName(current_edge_id_) + " blocked by an obstacle" +
                      (far_side_next ? "; will check it from the other end" : "") +
                      (cut_off > 0.0 ? "; blocked at both ends, so the " +
                                           std::to_string(static_cast<int>(std::round(cut_off))) +
                                           " m between the two obstacles can't be checked"
                                     : "") +
                      " - " + progressText());

        obstacle_avoidance_active_ = true; // the obstacle stays in the stop range while we turn away from it
        driveBackToJunction();
    }

    // Waypoints before current_waypoint_index_ have been reached; drive them in reverse, ending at the junction
    // this traversal started from. onEdgeFinished() then replans from there.
    void MissionManager::driveBackToJunction()
    {
        const size_t reached = std::max<size_t>(current_waypoint_index_, 1);
        std::vector<Waypoint> back(current_waypoints_.begin(), current_waypoints_.begin() + reached);
        std::reverse(back.begin(), back.end());
        current_waypoints_ = back;
        current_waypoint_index_ = 0;
        transit_to_edge_start_ = false;
        goal_retry_count_ = 0;
        backtracking_ = true;
        sendNextWaypoint();
    }

    // Mark the current edge blocked where the robot is now, and put a marker on the GUI map. If it was being
    // inspected for the first time, the stretch beyond the obstacle is still uninspected, so it is to be
    // inspected later from the other end (returns true); if we were already coming from that other end
    // (or only passing through), there is nothing more to check on it.
    bool MissionManager::markCurrentEdgeBlocked()
    {
        const TrailEdge *edge = graph_.getEdge(current_edge_id_);
        const bool first_time = edge && edge->status == EdgeStatus::UNINSPECTED;
        graph_.setEdgeStatus(current_edge_id_, EdgeStatus::BLOCKED);
        graph_.setFarSideNode(current_edge_id_, first_time ? planner_.otherEnd(current_edge_id_, current_node_id_) : -1);
        recordBlockedPoint();
        publishProgress();
        return first_time;
    }

    // Checking a blocked edge from its far side and stopped by an obstacle again: the robot has now been blocked
    // coming from each end, so the stretch of trail between the two obstacles can't be reached from either side.
    // Flag both obstacles and record that stretch on the edge, for the GUI. Returns its length (m), or 0 if the
    // two are the same obstacle seen from each side (recordBlockedPoint merged them, or they are very close).
    double MissionManager::flagInaccessibleSection()
    {
        const int far_end = planner_.otherEnd(current_edge_id_, current_node_id_);
        auto found_from = [this](int node)
        {
            return std::find_if(blocked_points_.begin(), blocked_points_.end(), [&](const BlockedPoint &b)
                                { return b.edge == current_edge_id_ && b.from_node == node && !b.avoided; });
        };
        const auto near = found_from(current_node_id_);
        const auto far = found_from(far_end);
        if (near == blocked_points_.end() || far == blocked_points_.end() || near == far)
        {
            RCLCPP_INFO(get_logger(), "Edge %d is blocked by the same obstacle from both ends", current_edge_id_);
            return 0.0;
        }
        double start = graph_.distanceAlongEdge(current_edge_id_, near->x, near->y);
        double end = graph_.distanceAlongEdge(current_edge_id_, far->x, far->y);
        if (start > end)
        {
            std::swap(start, end);
        }
        if (end - start < inaccessible_min_gap_)
        {
            RCLCPP_INFO(get_logger(), "Edge %d: obstacles from each end are %.1f m apart, taken to be the same one",
                        current_edge_id_, end - start);
            return 0.0;
        }
        graph_.setInaccessibleSection(current_edge_id_, graph_.edgeSection(current_edge_id_, start, end));
        near->bounds_gap = true;
        far->bounds_gap = true;
        RCLCPP_WARN(get_logger(), "Edge %d blocked at both ends: obstacles at (%.2f, %.2f) and (%.2f, %.2f), "
                                  "%.1f m between them can't be inspected",
                    current_edge_id_, far->x, far->y, near->x, near->y, end - start);
        publishBlockedPoints();
        publishGraph();
        return end - start;
    }

    // The obstacle is about blocked_marker_ahead_ in front of the robot when it stops for it. avoided: Nav2 is
    // going around it rather than the trail being blocked. An obstacle already marked on this edge (stopped for
    // again while going round, or going round failed and we backtrack) keeps its one marker; blocked wins.
    void MissionManager::recordBlockedPoint(bool avoided)
    {
        try
        {
            const auto tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            const auto &q = tf.transform.rotation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            BlockedPoint point;
            point.edge = current_edge_id_;
            point.x = tf.transform.translation.x + blocked_marker_ahead_ * std::cos(yaw);
            point.y = tf.transform.translation.y + blocked_marker_ahead_ * std::sin(yaw);
            point.from_node = current_node_id_;
            point.avoided = avoided;
            const auto same = std::find_if(blocked_points_.begin(), blocked_points_.end(), [&](const BlockedPoint &b)
                                           { return b.edge == point.edge && std::hypot(b.x - point.x, b.y - point.y) < 1.5; });
            if (same != blocked_points_.end())
            {
                same->avoided = same->avoided && avoided;
            }
            else
            {
                blocked_points_.push_back(point);
            }
            RCLCPP_INFO(get_logger(), "Edge %d %s at (%.2f, %.2f), coming from %s", point.edge,
                        avoided ? "has an obstacle (going around it)" : "blocked", point.x, point.y,
                        junctionName(point.from_node).c_str());
            publishBlockedPoints();
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get robot pose to mark where edge %d is blocked: %s",
                        current_edge_id_, ex.what());
        }
    }

} // namespace trail_decision_making
