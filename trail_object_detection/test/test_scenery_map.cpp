// Plain-assert unit tests for the known-scenery map. A synthetic grid checks the geometry; the real
// Phase 1 map (41068_ignition_bringup/maps/trail_map.yaml) checks that the trees which reach over the
// trail count as scenery while the rock obstacle_world adds on the trail does not.
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "trail_object_detection/scenery_map.hpp"

using namespace trail_object_detection;

#ifndef TRAIL_MAP_YAML
#define TRAIL_MAP_YAML "../41068_ignition_bringup/maps/trail_map.yaml"
#endif

int main()
{
  // 10 x 10 cells of 0.1 m, origin (-0.5, -0.5): one occupied cell at grid (7, 2) => centre (0.25, -0.25)
  std::vector<bool> grid(100, false);
  grid[2 * 10 + 7] = true;
  SceneryMap synthetic;
  synthetic.setGrid(10, 10, 0.1, -0.5, -0.5, grid, 0.2);
  assert(synthetic.loaded() && synthetic.occupiedCells() == 1);
  assert(synthetic.isScenery(0.25, -0.25));   // the cell itself
  assert(synthetic.isScenery(0.25, -0.05));   // 0.2 m away: within tolerance
  assert(!synthetic.isScenery(0.25, 0.25));   // 0.5 m away
  assert(!synthetic.isScenery(-0.25, -0.25)); // mirrored in x: rows/columns not swapped
  assert(!synthetic.isScenery(5.0, 5.0));     // outside the map: unknown, not scenery
  assert(!SceneryMap().isScenery(0.0, 0.0));  // nothing loaded

  // The real Phase 1 map
  SceneryMap map;
  std::string error;
  const bool ok = map.load(TRAIL_MAP_YAML, 0.35, error);
  if (!ok)
  {
    std::cerr << "could not load " << TRAIL_MAP_YAML << ": " << error << "\n";
    return 1;
  }
  // Trees whose low foliage reaches over the trail (trail_world.sdf): scenery
  const double trees[][2] = {{-2.0, 4.0}, {7.0, 2.0}, {-3.0, -6.6}, {-5.4, 0.2}, {-1.2, 0.3}};
  for (const auto &t : trees)
  {
    bool near = false;
    for (double dx = -0.3; dx <= 0.3 && !near; dx += 0.1)
    {
      for (double dy = -0.3; dy <= 0.3 && !near; dy += 0.1)
      {
        near = map.isScenery(t[0] + dx, t[1] + dy);
      }
    }
    assert(near);
  }
  // obstacle_world's Brown_Rock, on trail 1: NOT scenery, so it still stops the robot
  assert(!map.isScenery(0.8, 5.2));
  // Open trail with nothing beside it (trail 1 waypoint, trail 3 loop): not scenery
  assert(!map.isScenery(3.1, 5.8));
  assert(!map.isScenery(-5.2, -4.4));

  std::cout << "test_scenery_map: all passed (" << map.occupiedCells() << " occupied cells in trail_map)\n";
  return 0;
}
