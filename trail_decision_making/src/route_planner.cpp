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

    std::vector<int> RoutePlanner::planToNextUninspected(int start_node) const
    {
        // Breadth-first search: fewest edges to reach a node with an uninspected edge.
        // Swap for Dijkstra (weight = trail length) once BFS works.
        std::queue<int> frontier;
        std::unordered_map<int, std::pair<int, int>> parent; // node -> (prev node, edge used)
        frontier.push(start_node);
        parent[start_node] = {-1, -1};

        while (!frontier.empty())
        {
            const int node = frontier.front();
            frontier.pop();

            for (int edge_id : graph_.getConnectedEdges(node))
            {
                const TrailEdge *e = graph_.getEdge(edge_id);
                if (!e || e->status == EdgeStatus::BLOCKED)
                {
                    continue;
                }

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

                const int next = otherEnd(edge_id, node);
                if (parent.find(next) == parent.end())
                {
                    parent[next] = {node, edge_id};
                    frontier.push(next);
                }
            }
        }
        return {};
    }

} // namespace trail_decision_making