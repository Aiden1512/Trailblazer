#include "trail_decision_making/route_planner.h"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <utility>

namespace trail_decision_making
{
    // Node reached after traversing edge_id starting from node_id
    int RoutePlanner::otherEnd(int edge_id, int node_id) const
    {
        const TrailEdge *e = graph_.getEdge(edge_id); // trails are bidirectional, so return the other node of the edge
        if (!e)
        {
            return -1;
        }
        return (e->from_node == node_id) ? e->to_node : e->from_node; // trails are bidirectional
    }

    // Plan a route from start_node to the nearest uninspected edge, returning a sequence of edge ids to traverse.
    // If no uninspected edges are reachable, return an empty vector.
    std::vector<int> RoutePlanner::planToNextUninspected(int start_node) const
    {
        // Breadth-first search: fewest edges to reach a node with an uninspected edge.
        // Swap for Dijkstra (weight = trail length) once BFS works.
        std::queue<int> frontier;
        std::unordered_map<int, std::pair<int, int>> parent; // node -> (prev node, edge used)
        frontier.push(start_node);
        parent[start_node] = {-1, -1};

        // BFS loop
        while (!frontier.empty())
        {
            const int node = frontier.front(); // get the next node to explore
            frontier.pop(); // remove it from the frontier

            // Check all edges connected to this node
            for (int edge_id : graph_.getConnectedEdges(node))
            {
                const TrailEdge *e = graph_.getEdge(edge_id); // get the edge from the graph
                if (!e || e->status == EdgeStatus::BLOCKED) // skip blocked edges
                {
                    continue;
                }
                // If this edge is uninspected, we have found our goal. 
                //Reconstruct the path and return it.
                if (e->status == EdgeStatus::UNINSPECTED) 
                {
                    std::vector<int> path{edge_id};
                    for (int cur = node; parent[cur].second != -1; cur = parent[cur].first)
                    {
                        path.push_back(parent[cur].second);
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
                // If this edge is inspected, add the other end of the edge to the frontier if we haven't seen it yet.
                const int next = otherEnd(edge_id, node);
                if (parent.find(next) == parent.end())
                {
                    parent[next] = {node, edge_id}; // record how we got to this node
                    frontier.push(next);
                }
            }
        }
        return {}; // no uninspected edges reachable
    }

} // namespace trail_decision_making