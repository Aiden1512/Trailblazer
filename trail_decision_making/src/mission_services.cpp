// The GUI's mission controls: start, e-stop, resume, return home, obstacle mode and trail source.
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

    // Called when the operator presses "Start Autonomous Inspection" in the GUI. Checks the
    // trail graph can be loaded, then hands over to the start timer, which retries startMission()
    // every second until localisation is available. Progress after this goes out on mission_status.
    void MissionManager::onStartRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (mission_active_)
        {
            response->success = false;
            response->message = "Mission is already running";
            return;
        }
        if (start_timer_ && !start_timer_->is_canceled())
        {
            response->success = false;
            response->message = "Mission is already starting";
            return;
        }
        loadGraph(); // check the graph now so a bad waypoints file is reported straight back to the GUI
        if (graph_.edges().empty())
        {
            response->success = false;
            response->message = "Cannot start: no trail graph loaded from '" + waypoints_file_ + "'";
            publishStatus("Cannot start: trail graph has no edges");
            return;
        }

        // Clear anything left over from a previous (finished or stopped) run
        route_.clear();
        current_goal_handle_.reset();
        follow_goal_handle_.reset();
        line_goal_sent_ = false;
        going_around_ = false;
        obstacle_avoidance_active_ = false;
        inspecting_edge_ = false;
        backtracking_ = false;
        returning_home_ = false;
        driving_to_home_pose_ = false;
        paused_ = false;
        home_requested_ = false;
        failed_recoveries_ = 0;
        checking_far_side_ = false;
        blocked_points_.clear();
        publishBlockedPoints();

        RCLCPP_INFO(get_logger(), "Start command received");
        publishStatus("Start command received");
        start_timer_ = create_wall_timer(1s, [this]()
                                         { startMission(); });
        response->success = true;
        response->message = "Mission starting (" + std::to_string(graph_.edges().size()) + " trail edges)";
    }

    // The GUI's E-STOP: cancel whatever Nav2 is doing so the robot stops where it is, and send nothing more
    // until Resume or Return Home. The mission state (edge, waypoint, route) is kept as it was.
    void MissionManager::onPauseRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No mission is running";
            return;
        }
        if (paused_)
        {
            response->success = false;
            response->message = "Mission is already stopped";
            return;
        }
        paused_ = true;
        haltRobot();
        RCLCPP_WARN(get_logger(), "E-STOP from the GUI: robot stopped, mission paused");
        publishStatus("E-STOP: robot stopped - " + progressText() + ". Resume or Return Home to carry on");
        response->success = true;
        response->message = "E-stop: robot stopped";
    }

    void MissionManager::onResumeRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No mission is running";
            return;
        }
        if (!paused_)
        {
            response->success = false;
            response->message = "Mission is not stopped";
            return;
        }
        RCLCPP_INFO(get_logger(), "Resume from the GUI");
        publishStatus("Resuming mission - " + progressText());
        resumeDriving();
        response->success = true;
        response->message = "Mission resumed";
    }

    // The GUI's obstacle mode toggle. Only sets backtrack_on_obstacle, which is read each time an obstacle is
    // handled, so it applies from the next obstacle (or the one being waited on). Going around, if Nav2 still
    // can't get past, the trail is marked blocked and we backtrack anyway (handleNavigationFailure).
    void MissionManager::onObstacleModeRequest(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        set_parameter(rclcpp::Parameter("backtrack_on_obstacle", !request->data));
        response->success = true;
        response->message = request->data
                                ? "Obstacle mode: Nav2 plans around obstacles (backtracks if it can't get past within " +
                                      std::to_string(static_cast<int>(std::round(go_around_timeout_sec_))) + " s)"
                                : "Obstacle mode: mark the trail blocked and backtrack to the previous junction";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    // The GUI's trail source toggle: swap between the pre-recorded trail file and the one Phase 1 records, so both
    // can be inspected in one run without restarting. Reloads the graph straight away so the GUI shows it.
    void MissionManager::onTrailSourceRequest(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (mission_active_ || (start_timer_ && !start_timer_->is_canceled()))
        {
            response->success = false;
            response->message = "Can't change the trails while a mission is running";
            return;
        }
        const std::string &file = request->data ? phase1_waypoints_file_ : recorded_waypoints_file_;
        if (file.empty())
        {
            response->success = false;
            response->message = std::string("No ") + (request->data ? "phase1_waypoints_file" : "waypoints_file") +
                                " set in mission_manager.yaml";
            return;
        }
        waypoints_file_ = file;
        use_phase1_trails_ = request->data;
        loadGraph();
        blocked_points_.clear(); // they belong to the old trails
        publishBlockedPoints();
        publishProgress(); // also republishes the graph
        const char *source = use_phase1_trails_ ? "Phase 1" : "recorded";
        if (graph_.edges().empty())
        {
            response->success = false;
            response->message = std::string("Switched to the ") + source + " trails, but none could be loaded from '" +
                                file + "'";
            return;
        }
        response->success = true;
        response->message = std::string("Using the ") + source + " trails from '" + file + "' (" +
                            std::to_string(graph_.edges().size()) + " trail edges)";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    // The GUI's Return Home: skip every trail still to inspect and follow the trails home from here.
    // Also works while e-stopped (it resumes, heading home).
    void MissionManager::onReturnHomeRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No mission is running";
            return;
        }
        if (returning_home_) // already on the way (all inspected, or asked before): just make sure we're moving
        {
            response->success = true;
            response->message = paused_ ? "Resuming: already heading home" : "Already heading home";
            if (paused_)
            {
                publishStatus("Resuming: heading home - " + progressText());
                resumeDriving();
            }
            return;
        }

        haltRobot(); // stop the current goal; one heading home replaces it below
        clearPause();
        if (line_goal_sent_)
        {
            updateWaypointIndexFromPose(); // how far along the trail the robot got
            line_goal_sent_ = false;
        }
        const size_t skipped = graph_.countEdges(EdgeStatus::UNINSPECTED) + graph_.pendingFarSides();
        returning_home_ = true;
        home_requested_ = true;
        route_.clear(); // chooseNextEdgeAndSendGoal() plans home instead
        RCLCPP_WARN(get_logger(), "Return home from the GUI: skipping %zu trails still to inspect", skipped);
        publishStatus("Return home requested: skipping " + std::to_string(skipped) + " trails still to inspect");
        response->success = true;
        response->message = "Returning home (skipping " + std::to_string(skipped) + " trails)";

        if (backtracking_ || driving_to_home_pose_)
        {
            sendNextWaypoint(); // already heading for a junction (or home itself); the plan home goes on from there
            return;
        }
        if (!onTrailEdge()) // off the network, driving to a trail's start: go home from the nearest junction
        {
            replanFromNearestJunction();
            return;
        }

        // Part way along a trail: finish it or turn back, whichever end gets us home sooner along the trails.
        // A blocked trail can't be driven through, so turn back from that one.
        const auto &w = current_waypoints_;
        const size_t k = std::clamp<size_t>(current_waypoint_index_, 1, w.size());
        double back = 0.0, ahead = 0.0;
        for (size_t i = 1; i < w.size(); ++i)
        {
            (i < k ? back : ahead) += std::hypot(w[i].x - w[i - 1].x, w[i].y - w[i - 1].y);
        }
        back += distanceHomeFrom(current_node_id_);
        ahead = graph_.getEdgeStatus(current_edge_id_) == EdgeStatus::BLOCKED
                    ? std::numeric_limits<double>::infinity()
                    : ahead + distanceHomeFrom(planner_.otherEnd(current_edge_id_, current_node_id_));
        going_around_ = false;
        checking_far_side_ = false;
        if (back <= ahead)
        {
            inspecting_edge_ = false; // turning back, so the trail stays uninspected
            RCLCPP_INFO(get_logger(), "Turning back along edge %d to %s to head home", current_edge_id_,
                        junctionName(current_node_id_).c_str());
            driveBackToJunction();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Finishing edge %d, then heading home", current_edge_id_);
            sendNextWaypoint();
        }
    }

    // Stop the robot where it is: cancel the Nav2 goals and anything waiting to send a new one
    void MissionManager::haltRobot()
    {
        if (retry_timer_)
        {
            retry_timer_->cancel();
        }
        if (obstacle_wait_timer_)
        {
            obstacle_wait_timer_->cancel();
        }
        if (go_around_timer_)
        {
            go_around_timer_->cancel();
        }
        obstacle_pause_active_ = false;
        cancelGoals();
    }

    // Leave the e-stop. Forgets any obstacle seen before or during the stop, so the next obstacle_stop message
    // (sent every lidar frame) stops us for it again rather than driving into it.
    void MissionManager::clearPause()
    {
        paused_ = false;
        if (!obstacle_avoidance_active_) // going around, it is ignored until it clears anyway
        {
            obstacle_stop_active_ = false;
        }
        goal_retry_count_ = 0;
    }

    // Carry on after an e-stop from where the robot is (a trail line is resent from the nearest waypoint).
    void MissionManager::resumeDriving()
    {
        clearPause();
        if (going_around_)
        {
            startGoAroundTimer(); // the e-stop cancelled it; give the go-around its full time again
        }
        sendNextWaypoint();
    }

} // namespace trail_decision_making
