#include "trail_decision_making/route_planner.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace trail_decision_making
{
    // Node reached after traversing edge_id starting from node_id
    int RoutePlanner::otherEnd(int edge_id, int node_id) const
    {
        const TrailEdge *e = graph_.getEdge(edge_id);
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
                if (!e)
                {
                    continue;
                }
                // A blocked edge is a goal only from its unchecked far side (inspect it up to the obstacle),
                // and is never driven through.
                const bool far_side_check = e->status == EdgeStatus::BLOCKED && e->far_side_node == node;
                if (e->status == EdgeStatus::BLOCKED && !far_side_check)
                {
                    continue;
                }
                // Found the goal: walk the parents back to start_node to build the route
                if (e->status == EdgeStatus::UNINSPECTED || far_side_check)
                {
                    std::vector<int> path{edge_id};
                    for (int cur = node; parent[cur].second != -1; cur = parent[cur].first)
                    {
                        path.push_back(parent[cur].second);
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
                // Already inspected: keep searching beyond it
                const int next = otherEnd(edge_id, node);
                if (parent.find(next) == parent.end())
                {
                    parent[next] = {node, edge_id};
                    frontier.push(next);
                }
            }
        }
        return {}; // no uninspected edges reachable
    }

    // Plan a route from start_node to goal_node (e.g. back home), returning a sequence of edge ids to traverse.
    // If goal_node is start_node or can't be reached, return an empty vector.
    std::vector<int> RoutePlanner::planToNode(int start_node, int goal_node) const
    {
        // Same breadth-first search as planToNextUninspected, but any non-blocked edge may be used
        std::queue<int> frontier;
        std::unordered_map<int, std::pair<int, int>> parent; // node -> (prev node, edge used)
        frontier.push(start_node);
        parent[start_node] = {-1, -1};

        while (!frontier.empty())
        {
            const int node = frontier.front();
            frontier.pop();
            if (node == goal_node) // reconstruct the path back to start_node
            {
                std::vector<int> path;
                for (int cur = node; parent[cur].second != -1; cur = parent[cur].first)
                {
                    path.push_back(parent[cur].second);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            for (int edge_id : graph_.getConnectedEdges(node))
            {
                const TrailEdge *e = graph_.getEdge(edge_id);
                if (!e || e->status == EdgeStatus::BLOCKED)
                {
                    continue;
                }
                const int next = otherEnd(edge_id, node);
                if (parent.find(next) == parent.end())
                {
                    parent[next] = {node, edge_id};
                    frontier.push(next);
                }
            }
        }
        return {}; // goal not reachable
    }

    // 0-1 breadth-first search: a blocked edge costs 1 and any other edge 0, so the route found goes through
    // as few blocked trails as possible (used to get home when every trail-only route is blocked).
    std::vector<int> RoutePlanner::planToNodeThroughBlocked(int start_node, int goal_node) const
    {
        std::deque<int> frontier;
        std::unordered_map<int, int> cost;                   // node -> blocked edges used to reach it
        std::unordered_map<int, std::pair<int, int>> parent; // node -> (prev node, edge used)
        frontier.push_back(start_node);
        cost[start_node] = 0;
        parent[start_node] = {-1, -1};

        while (!frontier.empty())
        {
            const int node = frontier.front();
            frontier.pop_front();
            for (int edge_id : graph_.getConnectedEdges(node))
            {
                const TrailEdge *e = graph_.getEdge(edge_id);
                if (!e)
                {
                    continue;
                }
                const int edge_cost = e->status == EdgeStatus::BLOCKED ? 1 : 0;
                const int next_node = otherEnd(edge_id, node);
                const int new_cost = cost[node] + edge_cost;
                const auto known = cost.find(next_node); // already reached at least as cheaply: skip
                if (known != cost.end() && known->second <= new_cost)
                {
                    continue;
                }
                cost[next_node] = new_cost;
                parent[next_node] = {node, edge_id};
                // 0-cost edges go to the front of the deque so nodes still come off in cost order
                if (edge_cost == 0)
                {
                    frontier.push_front(next_node);
                }
                else
                {
                    frontier.push_back(next_node);
                }
            }
        }
        if (start_node == goal_node || cost.find(goal_node) == cost.end())
        {
            return {};
        }
        std::vector<int> path;
        for (int cur = goal_node; parent[cur].second != -1; cur = parent[cur].first)
        {
            path.push_back(parent[cur].second);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

} // namespace trail_decision_making