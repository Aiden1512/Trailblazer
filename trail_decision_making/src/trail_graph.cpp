#include "trail_decision_making/trail_graph.h"

#include <cmath>
#include <limits>

#include "yaml-cpp/yaml.h"

namespace trail_decision_making
{

    namespace
    {
        constexpr double kNodeMergeTolerance = 0.5; // metres, tune to your mapping output
    }

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
                if (!wps || !wps.IsSequence() || wps.size() < 2)
                {
                    continue;
                }

                TrailEdge edge;
                edge.trail_name = trail["name"] ? trail["name"].as<std::string>() : "";

                for (const auto &wp : wps)
                {
                    edge.waypoints.push_back(
                        {wp["x"].as<double>(), wp["y"].as<double>(), wp["yaw"].as<double>(0.0)});
                }

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
        return !edges_.empty();
    }

    const TrailEdge *TrailGraph::getEdge(int edge_id) const
    {
        for (const auto &e : edges_)
        {
            if (e.id == edge_id)
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
            if (e.from_node == node_id || e.to_node == node_id)
            {
                result.push_back(e.id);
            }
        }
        return result;
    }
    // Set the status of an edge by its id. Does nothing if the id is invalid.
    void TrailGraph::setEdgeStatus(int edge_id, EdgeStatus status)
    {
        for (auto &e : edges_)
        {
            if (e.id == edge_id)
            {
                e.status = status;
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

    int TrailGraph::findNearestNode(double x, double y) const
    {
        int best = -1;
        double best_d2 = std::numeric_limits<double>::max();
        for (const auto &n : nodes_)
        {
            const double dx = n.position.x - x;
            const double dy = n.position.y - y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)
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
        return static_cast<double>(done) / static_cast<double>(edges_.size()); 
    }
    
    void TrailGraph::resetAllEdgesToUninspected()
    {
        for (auto &e : edges_)
        {
            e.status = EdgeStatus::UNINSPECTED;
        }
    }

    int TrailGraph::getOrCreateNode(double x, double y, double yaw)
    {
        const int nearest = findNearestNode(x, y);
        if (nearest >= 0)
        {
            const auto &n = nodes_[nearest];
            if (std::hypot(n.position.x - x, n.position.y - y) < kNodeMergeTolerance)
            {
                return nearest;
            }
        }
        TrailNode node;
        node.id = static_cast<int>(nodes_.size());
        node.position = {x, y, yaw};
        nodes_.push_back(node);
        return node.id;
    }

} // namespace trail_decision_making