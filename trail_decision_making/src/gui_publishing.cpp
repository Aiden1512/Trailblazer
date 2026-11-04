// What the GUI is sent: mission status text, progress, the trail graph and blocked points.
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

    // JSON list for the GUI: [{"edge": 2, "x": -5.1, "y": -1.2, "from": "junction 3", "avoided": false,
    // "bounds_gap": false}, ...] in the map frame (metres)
    void MissionManager::publishBlockedPoints()
    {
        std::ostringstream json;
        json << std::fixed << std::setprecision(2) << "[";
        for (size_t i = 0; i < blocked_points_.size(); ++i)
        {
            const auto &b = blocked_points_[i];
            const TrailEdge *edge = graph_.getEdge(b.edge);
            json << (i ? ", " : "") << "{\"edge\": " << b.edge << ", \"trail\": " << (edge ? edge->trail_id : -1) << ", \"x\": " << b.x << ", \"y\": " << b.y
                 << ", \"from\": \"" << junctionName(b.from_node) << "\", \"avoided\": " << (b.avoided ? "true" : "false")
                 << ", \"bounds_gap\": " << (b.bounds_gap ? "true" : "false") << "}";
        }
        json << "]";
        std_msgs::msg::String msg;
        msg.data = json.str();
        blocked_points_pub_->publish(msg);
    }

    // Completion = trails that have been resolved (inspected, or found blocked) out of all trails
    void MissionManager::publishProgress()
    {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(graph_.explorationPercentage() * 100.0);
        progress_pub_->publish(msg);
        publishGraph(); // trail statuses change exactly when progress does
    }

    // JSON for the GUI (map frame, metres):
    // {"file": ..., "phase1": false, "junctions": [{"node": 0, "junction": 1, "x": .., "y": ..}, ...],
    //  "trails": [{"edge": 0, "trail": 1, "from": 0, "to": 1, "status": "inspected", "far_side": -1,
    //              "waypoints": [[x, y], ...], "inaccessible": [[x, y], ...]}, ...]}
    // "junction" is -1 for a trail's dead end; status is unvisited / inspected / blocked. "inaccessible" is the
    // stretch between two obstacles of a trail blocked at both ends, empty otherwise.
    void MissionManager::publishGraph()
    {
        std::ostringstream json;
        json << std::fixed << std::setprecision(3) << "{\"file\": \"" << waypoints_file_ << "\", \"phase1\": "
             << (use_phase1_trails_ ? "true" : "false") << ", \"junctions\": [";
        const auto &nodes = graph_.nodes();
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            json << (i ? ", " : "") << "{\"node\": " << nodes[i].id << ", \"junction\": " << nodes[i].junction_id
                 << ", \"x\": " << nodes[i].position.x << ", \"y\": " << nodes[i].position.y << "}";
        }
        json << "], \"trails\": [";
        const auto &edges = graph_.edges();
        for (size_t i = 0; i < edges.size(); ++i)
        {
            const TrailEdge &e = edges[i];
            const char *status = e.status == EdgeStatus::INSPECTED ? "inspected"
                                 : e.status == EdgeStatus::BLOCKED ? "blocked"
                                                                   : "unvisited";
            json << (i ? ", " : "") << "{\"edge\": " << e.id << ", \"trail\": " << e.trail_id << ", \"from\": "
                 << e.from_node << ", \"to\": " << e.to_node << ", \"status\": \"" << status
                 << "\", \"far_side\": " << e.far_side_node << ", \"waypoints\": [";
            for (size_t w = 0; w < e.waypoints.size(); ++w)
            {
                json << (w ? ", " : "") << "[" << e.waypoints[w].x << ", " << e.waypoints[w].y << "]";
            }
            json << "], \"inaccessible\": [";
            for (size_t w = 0; w < e.inaccessible.size(); ++w)
            {
                json << (w ? ", " : "") << "[" << e.inaccessible[w].x << ", " << e.inaccessible[w].y << "]";
            }
            json << "]}";
        }
        json << "]}";
        std_msgs::msg::String msg;
        msg.data = json.str();
        graph_pub_->publish(msg);
    }

    // "Trail 3" (the trail's id in the YAML file, as on the GUI map); "trail 3" when capital is false
    std::string MissionManager::trailName(int edge_id, bool capital) const
    {
        const TrailEdge *edge = graph_.getEdge(edge_id);
        const int id = (edge && edge->trail_id >= 0) ? edge->trail_id : edge_id;
        return std::string(capital ? "Trail " : "trail ") + std::to_string(id);
    }

    std::string MissionManager::progressText() const
    {
        return std::to_string(static_cast<int>(std::round(graph_.explorationPercentage() * 100.0))) + "% complete";
    }

    std::string MissionManager::junctionName(int node_id) const
    {
        if (node_id < 0 || node_id >= static_cast<int>(graph_.nodes().size()))
        {
            return "unknown junction";
        }
        const int junction_id = graph_.nodes()[node_id].junction_id;
        return junction_id >= 0 ? "junction " + std::to_string(junction_id)
                                : "trail end " + std::to_string(node_id);
    }

    void MissionManager::publishStatus(const std::string &text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        status_pub_->publish(msg);
    }

} // namespace trail_decision_making
