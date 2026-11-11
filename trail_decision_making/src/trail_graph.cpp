#include "trail_decision_making/trail_graph.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace trail_decision_making
{
    namespace
    {
        // Trail ends closer than this (m) to an existing node are joined to it rather than getting their own
        constexpr double kNodeMergeTolerance = 0.5;
    } // namespace

    // Load a trail file (path_recorder's junction_nodes.yaml format). Returns false on failure.
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

            // Junctions recorded by the mapping team become the graph nodes, and trails are
            // joined to them by start_junction/end_junction. Trail waypoints start and end a
            // metre or two away from the junction, so they can't be joined by position alone.
            std::unordered_map<int, int> junction_nodes; // YAML junction id -> node id
            const YAML::Node junctions = root["junctions"];
            if (junctions && junctions.IsSequence())
            {
                for (const auto &junction : junctions)
                {
                    const int node_id = getOrCreateNode(
                        junction["x"].as<double>(), junction["y"].as<double>(), junction["yaw"].as<double>(0.0));
                    nodes_[node_id].junction_id = junction["id"].as<int>();
                    junction_nodes[junction["id"].as<int>()] = node_id;
                }
            }
            // node id for a trail's start_junction/end_junction, -1 if it is null or unknown
            auto junctionNode = [&junction_nodes](const YAML::Node &key)
            {
                if (!key || key.IsNull())
                {
                    return -1;
                }
                const auto it = junction_nodes.find(key.as<int>());
                return it == junction_nodes.end() ? -1 : it->second;
            };

            for (const auto &trail : trails)
            {
                const YAML::Node wps = trail["waypoints"];
                if (!wps || !wps.IsSequence())
                {
                    continue;
                }

                TrailEdge edge;
                edge.trail_name = trail["name"] ? trail["name"].as<std::string>() : "";
                edge.trail_id = trail["id"] ? trail["id"].as<int>(-1) : -1;

                for (const auto &wp : wps)
                {
                    edge.waypoints.push_back(
                        {wp["x"].as<double>(), wp["y"].as<double>(), wp["yaw"].as<double>(0.0)}); // default yaw to 0.0 if not provided
                }
                // Drive all the way into the junctions at each end, so the robot is at the junction when the edge is done
                const int start_node = junctionNode(trail["start_junction"]);
                const int end_node = junctionNode(trail["end_junction"]);
                if (start_node >= 0)
                {
                    edge.waypoints.insert(edge.waypoints.begin(), nodes_[start_node].position);
                }
                if (end_node >= 0)
                {
                    edge.waypoints.push_back(nodes_[end_node].position);
                }
                if (edge.waypoints.size() < 2) // need at least two waypoints to form an edge
                {
                    continue;
                }
                // Ends without a junction (old files, or a dead-end trail) get a node at the first/last waypoint,
                // reusing existing nodes if they are within the merge tolerance
                const Waypoint &first = edge.waypoints.front();
                const Waypoint &last = edge.waypoints.back();
                edge.from_node = start_node >= 0 ? start_node : getOrCreateNode(first.x, first.y, first.yaw);
                edge.to_node = end_node >= 0 ? end_node : getOrCreateNode(last.x, last.y, last.yaw);
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

    // Get a pointer to the edge with the given id, or nullptr if the id does not exist.
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
            if (d2 < best_d2)
            {
                best_d2 = d2;
                best = n.id;
            }
        }
        return best;
    }

    // Fraction of the trail network checked so far (shown as the GUI's progress), 0.0 if the graph is empty
    double TrailGraph::explorationPercentage() const
    {
        if (edges_.empty())
        {
            return 0.0;
        }
        double done = 0.0;
        for (const auto &e : edges_)
        {
            if (e.status == EdgeStatus::INSPECTED)
            {
                done += 1.0;
            }
            else if (e.status == EdgeStatus::BLOCKED)
            {
                done += e.far_side_node >= 0 ? 0.5 : 1.0; // only one side of the obstacle checked so far
            }
        }
        return done / static_cast<double>(edges_.size());
    }

    void TrailGraph::setFarSideNode(int edge_id, int node_id)
    {
        for (auto &e : edges_)
        {
            if (e.id == edge_id)
            {
                e.far_side_node = node_id;
                return;
            }
        }
    }

    size_t TrailGraph::pendingFarSides() const
    {
        return static_cast<size_t>(std::count_if(edges_.begin(), edges_.end(), [](const TrailEdge &e)
                                                 { return e.status == EdgeStatus::BLOCKED && e.far_side_node >= 0; }));
    }

    double TrailGraph::distanceAlongEdge(int edge_id, double x, double y) const
    {
        const TrailEdge *e = getEdge(edge_id);
        if (!e || e->waypoints.empty())
        {
            return 0.0;
        }
        const auto &w = e->waypoints;
        double best_d = std::hypot(x - w[0].x, y - w[0].y), best_s = 0.0, s = 0.0;
        for (size_t i = 1; i < w.size(); ++i)
        {
            // Find the closest point on the segment from w[i-1] to w[i] to (x, y)
            const double dx = w[i].x - w[i - 1].x, dy = w[i].y - w[i - 1].y;
            const double len = std::hypot(dx, dy);
            const double t = len > 1e-9 ? std::clamp(((x - w[i - 1].x) * dx + (y - w[i - 1].y) * dy) / (len * len), 0.0, 1.0) : 0.0;
            const double d = std::hypot(x - w[i - 1].x - t * dx, y - w[i - 1].y - t * dy);
            if (d < best_d)
            {
                best_d = d;
                best_s = s + t * len;
            }
            s += len;
        }
        return best_s;
    }
    // Get the waypoints along an edge between two distances from the from_node, interpolating the ends if necessary.
    std::vector<Waypoint> TrailGraph::edgeSection(int edge_id, double start, double end) const
    {
        std::vector<Waypoint> section;
        const TrailEdge *e = getEdge(edge_id);
        if (!e || e->waypoints.size() < 2 || end <= start)
        {
            return section;
        }
        const auto &w = e->waypoints;
        double s = 0.0;
        for (size_t i = 1; i < w.size(); ++i)
        {
            const double len = std::hypot(w[i].x - w[i - 1].x, w[i].y - w[i - 1].y);
            auto at = [&](double along)
            {
                const double t = len > 1e-9 ? (along - s) / len : 0.0;
                return Waypoint{w[i - 1].x + t * (w[i].x - w[i - 1].x), w[i - 1].y + t * (w[i].y - w[i - 1].y), w[i - 1].yaw};
            };
            if (section.empty() && start <= s + len)
            {
                section.push_back(at(start));
            }
            if (!section.empty())
            {
                if (end <= s + len)
                {
                    section.push_back(at(end));
                    return section;
                }
                section.push_back(w[i]);
            }
            s += len;
        }
        return section; // end was past the last waypoint: the section runs to it
    }

    void TrailGraph::setInaccessibleSection(int edge_id, const std::vector<Waypoint> &section)
    {
        for (auto &e : edges_)
        {
            if (e.id == edge_id)
            {
                e.inaccessible = section;
                return;
            }
        }
    }
    // Count the number of edges that have an inaccessible section (blocked at both ends)
    size_t TrailGraph::countInaccessibleSections() const
    {
        return static_cast<size_t>(std::count_if(edges_.begin(), edges_.end(),
                                                 [](const TrailEdge &e) { return !e.inaccessible.empty(); }));
    }

    // Number of edges with the given status
    size_t TrailGraph::countEdges(EdgeStatus status) const
    {
        return static_cast<size_t>(std::count_if(edges_.begin(), edges_.end(),
                                                 [status](const TrailEdge &e) { return e.status == status; }));
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