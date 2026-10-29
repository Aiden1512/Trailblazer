// Unit tests for the perception core. Plain asserts, no gtest or ROS needed.
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "trail_object_detection/obstacle_logic.hpp"

using namespace trail_object_detection;

int main()
{
  const Params p;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  // classify(): each class, with the sim numbers (ground at z = -0.80)
  assert(classify(nan, 0, 0, p) == PointClass::Invalid);
  assert(classify(20.f, 0, 0, p) == PointClass::Invalid);          // Past max_range
  assert(classify(0.3f, 0.0f, -0.5f, p) == PointClass::Self);      // Front of the chassis
  assert(classify(-0.3f, 0.0f, -0.5f, p) == PointClass::Self);     // Rear of the chassis
  assert(classify(0.3f, 0.5f, -0.5f, p) != PointClass::Self);      // Beside the body
  assert(classify(2.0f, 0.0f, -0.80f, p) == PointClass::Ground);   // Flat ground
  assert(classify(2.0f, 0.0f, -0.74f, p) == PointClass::Ground);   // 6 cm bump, ignored
  assert(classify(2.0f, 0.0f, -0.65f, p) == PointClass::Obstacle); // 15 cm plant
  assert(classify(2.0f, 0.0f, -0.30f, p) == PointClass::Obstacle); // Side of a rock
  assert(classify(2.0f, 0.0f, 0.04f, p) == PointClass::Obstacle);  // At mast height, would hit the robot
  assert(classify(2.0f, 0.0f, 0.30f, p) == PointClass::Overhead);  // Branch

  // Ground margin grows with range: at 6 m it is 0.24 m
  assert(classify(6.0f, 0.0f, -0.62f, p) == PointClass::Ground);
  assert(classify(6.0f, 0.0f, -0.50f, p) == PointClass::Obstacle);

  // countPoint(): distances are from the bumper, so ahead = x - 0.50
  ZoneCounts c;
  countPoint(1.2f, 0.0f, p, c); // 0.7 m clear, inside stop and warn
  assert(c.stop == 1 && c.warn == 1);
  c = ZoneCounts();
  countPoint(2.5f, 0.0f, p, c); // 2.0 m clear, warn only
  assert(c.stop == 0 && c.warn == 1);
  c = ZoneCounts();
  countPoint(4.2f, 0.0f, p, c); // Past warn_distance
  assert(c.stop == 0 && c.warn == 0);
  c = ZoneCounts();
  countPoint(1.0f, 0.8f, p, c); // Outside the lane
  assert(c.stop == 0 && c.warn == 0);
  c = ZoneCounts();
  countPoint(-1.0f, 0.0f, p, c); // Behind the robot
  assert(c.stop == 0 && c.warn == 0);

  // Debouncer(2, 3): on after 2 in a row, off after 3 in a row
  Debouncer d(2, 3);
  assert(!d.update(true));  // One positive is not enough
  assert(d.update(true));
  assert(d.update(false));
  assert(d.update(false));
  assert(!d.update(false));
  assert(!d.update(true));  // Lone glitch while off
  assert(!d.update(false));
  assert(!d.update(true));
  assert(d.update(true));
  assert(d.update(false));
  assert(d.update(true));   // Positive resets the release countdown
  assert(d.update(false));
  assert(d.update(false));
  assert(!d.update(false));

  // ObstacleScan: keeps the nearest point per bearing
  ObstacleScan s;
  s.add(2.0f, 0.0f);
  s.add(1.0f, 0.0f);
  s.add(0.0f, -3.0f);
  assert(std::fabs(s.ranges()[kScanBins / 2] - 1.0f) < 1e-3);      // Bearing 0
  assert(std::fabs(s.ranges()[kScanBins / 4] - 3.0f) < 1e-3);      // Bearing -90 deg
  assert(s.ranges()[0] == ObstacleScan::kEmpty);

  std::cout << "test_obstacle_logic: all assertions passed\n";
  return 0;
}
