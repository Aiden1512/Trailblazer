#include <gtest/gtest.h>

#include "trail_decision_making/route_planner.h"

// TODO: build a tiny TrailGraph in code (add a test-only setter or a small YAML
// fixture) and check: uninspected edge picked first, backtracking works,
// BLOCKED edges are avoided, empty result when everything is inspected.
TEST(RoutePlanner, EmptyGraphReturnsNoRoute)
{
    trail_decision_making::TrailGraph graph;
    trail_decision_making::RoutePlanner planner(graph);
    EXPECT_TRUE(planner.planToNextUninspected(0).empty());
}