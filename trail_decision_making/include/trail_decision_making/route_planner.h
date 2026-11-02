#ifndef TRAIL_DECISION_MAKING__ROUTE_PLANNER_HPP_
#define TRAIL_DECISION_MAKING__ROUTE_PLANNER_HPP_

#include <vector>

#include "trail_decision_making/trail_graph.h"

namespace trail_decision_making
{

    // Pure C++ (no ROS). Decides WHAT to inspect next; MissionManager decides HOW to drive it and trail_graph.cpp decides WHERE the waypoints are. This separation allows unit testing of the route planner without ROS.
    class RoutePlanner
    {
    public:
        explicit RoutePlanner(const TrailGraph &graph) : graph_(graph) {}

        // Ordered edge ids to drive from start_node: any backtracking edges through
        // already-inspected trail, ending with the nearest UNINSPECTED edge, or the nearest BLOCKED edge
        // reached at its far_side_node (to inspect it from that end up to the obstacle).
        // BLOCKED edges are never driven through. Empty vector => nothing left to inspect.
        std::vector<int> planToNextUninspected(int start_node) const;

        // Ordered edge ids to drive from start_node to goal_node, never using BLOCKED edges.
        // Empty vector => already there, or goal_node can't be reached.
        std::vector<int> planToNode(int start_node, int goal_node) const;

        // Same, but BLOCKED edges may be used: the route through the fewest of them.
        // Empty vector => already there, or goal_node isn't connected at all.
        std::vector<int> planToNodeThroughBlocked(int start_node, int goal_node) const;

        // Node reached after traversing edge_id starting from node_id
        int otherEnd(int edge_id, int node_id) const;

    private:
        const TrailGraph &graph_; // Reference to the graph to plan on
    };

} // namespace trail_decision_making

#endif // TRAIL_DECISION_MAKING__ROUTE_PLANNER_HPP_