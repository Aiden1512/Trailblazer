#include "trail_decision_making/trail_graph.h"

#include <cmath>
#include <limits>

#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <utility>

namespace trail_decision_making
{

    // namespace
    // {
        constexpr double kNodeMergeTolerance = 0.5; // metres, tune to your mapping output
        std::pair<double, double> TrailGraph::pointToSegment(
            double px, double py, double ax, double ay, double bx, double by)
        {
            const double dx = bx - ax;
            const double dy = by - ay;
            const double len2 = dx * dx + dy * dy;
            double frac = 0.0;
            if (len2 > 1e-9)
            {
                frac = ((px - ax) * dx + (py - ay) * dy) / len2;
                frac = std::max(0.0, std::min(frac, 1.0));
            }
            const double cx = ax + frac * dx;
            const double cy = ay + frac * dy;
            return {std::hypot(px - cx, py - cy), frac};
        }
    // } // namespace

    // Load the YAML file produced by the mapping team. Returns false on failure.
    bool TrailGraph::loadFromYaml(const std::string &file_path)
    {
        nodes_.clear();
        edges_.clear(); 

        try
        {
            const YAML::Node root = YAML::LoadFile(file_path);
            const YAML::Node trails = root["trails"];
            if (!trails || !trails.IsSequence())
            {
                return false;
            }

            for (const auto &trail : trails)
            {
                const YAML::Node wps = trail["waypoints"];
                if (!wps || !wps.IsSequence() || wps.size() < 2) // need at least two waypoints to form an edge
                {
                    continue;
                }

                TrailEdge edge;
                edge.trail_name = trail["name"] ? trail["name"].as<std::string>() : ""; 

                for (const auto &wp : wps) 
                {
                    edge.waypoints.push_back(
                        {wp["x"].as<double>(), wp["y"].as<double>(), wp["yaw"].as<double>(0.0)}); // default yaw to 0.0 if not provided
                }
                // Create nodes for the first and last waypoints of the edge, reusing existing nodes if they are within the merge tolerance
                const Waypoint &first = edge.waypoints.front();
                const Waypoint &last = edge.waypoints.back();
                edge.from_node = getOrCreateNode(first.x, first.y, first.yaw);
                edge.to_node = getOrCreateNode(last.x, last.y, last.yaw);
                edge.id = static_cast<int>(edges_.size());
                edges_.push_back(std::move(edge));
            }
        }
        catch (const YAML::Exception &)
        {
            return false; 
        }
        return !edges_.empty(); // maybe return the edges instead of returning true/false, so the caller can check if the graph is empty
    }

    // Get a pointer to the edge with the given id, or nullptr if the id does not exist.
    const TrailEdge *TrailGraph::getEdge(int edge_id) const
    {
        for (const auto &e : edges_)
        {
            if (e.id == edge_id) // return a pointer to the edge if the id matches
            {
                return &e;
            }
        }
        return nullptr;
    }
    // All edges touching this node (trails are traversable in both directions)
    std::vector<int> TrailGraph::getConnectedEdges(int node_id) const
    {
        std::vector<int> result;
        for (const auto &e : edges_)
        {
            if (e.from_node == node_id || e.to_node == node_id) // if the edge connects to the node, add it to the result
            {
                result.push_back(e.id); // push the edge id into the result vector
            }
        }
        return result;
    }
    // Set the status of an edge by its id. Does nothing if the id is invalid.
    void TrailGraph::setEdgeStatus(int edge_id, EdgeStatus status)
    {
        for (auto &e : edges_)
        {
            if (e.id == edge_id) // if the edge id matches, set the status and return
            {
                e.status = status; // set the status of the edge to the new status
                return;
            }
        }
    }
    // Get the status of an edge by its id. Returns BLOCKED if the id is invalid.
    EdgeStatus TrailGraph::getEdgeStatus(int edge_id) const
    {
        const TrailEdge *e = getEdge(edge_id);
        return e ? e->status : EdgeStatus::BLOCKED;
    }

    // Returns the id of the nearest node to the given (x, y) position, or -1 if the graph is empty.
    int TrailGraph::findNearestNode(double x, double y) const
    {
        int best = -1;
        double best_d2 = std::numeric_limits<double>::max();
        for (const auto &n : nodes_)
        {
            const double dx = n.position.x - x;
            const double dy = n.position.y - y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) // if this node is closer than the best so far, update the best
            {
                best_d2 = d2;
                best = n.id;
            }
        }
        return best;
    }

    // Return the percentage of inspected edges (inspected + blocked) / total, 0.0 if graph is empty
    double TrailGraph::explorationPercentage() const
    {
        if (edges_.empty())
        {
            return 0.0;
        }
        size_t done = 0;
        for (const auto &e : edges_)
        {
            if (e.status != EdgeStatus::UNINSPECTED)
            {
                ++done;
            }
        }
        // return the fraction of edges that are inspected or blocked (important for UI)
        return static_cast<double>(done) / static_cast<double>(edges_.size()); 
    }
    
    // Reset all edges to UNINSPECTED status. This is useful for testing or restarting a mission.
    void TrailGraph::resetAllEdgesToUninspected()
    {
        for (auto &e : edges_)
        {
            e.status = EdgeStatus::UNINSPECTED;
        }
    }

    // Reuse an existing node within a tolerance, otherwise create a new one
    int TrailGraph::getOrCreateNode(double x, double y, double yaw)
    {
        const int nearest = findNearestNode(x, y);
        if (nearest >= 0)
        {
            const auto &n = nodes_[nearest];
            // If the nearest node is within the merge tolerance, return its id instead of creating a new node
            if (std::hypot(n.position.x - x, n.position.y - y) < kNodeMergeTolerance)
            {
                return nearest;
            }
        }
        // Create a new node if no existing node is close enough
        TrailNode node;
        node.id = static_cast<int>(nodes_.size());
        node.position = {x, y, yaw};
        nodes_.push_back(node);
        return node.id;
    }

} // namespace trail_decision_making