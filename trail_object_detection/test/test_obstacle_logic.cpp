// Plain-assert unit tests for the ROS-free perception core.
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

  // classify
  assert(classify(nan, 0, 0, p) == PointClass::Invalid);
  assert(classify(20.f, 0, 0, p) == PointClass::Invalid);                 // beyond max_range
  assert(classify(0.3f, 0.0f, -0.5f, p) == PointClass::Self);             // own chassis
  assert(classify(0.3f, 0.5f, -0.5f, p) != PointClass::Self);             // beside the body
  assert(classify(2.0f, 0.0f, -0.80f, p) == PointClass::Ground);          // flat ground
  assert(classify(2.0f, 0.0f, -0.74f, p) == PointClass::Ground);          // 6 cm bump
  assert(classify(2.0f, 0.0f, -0.65f, p) == PointClass::Obstacle);        // 15 cm plant
  assert(classify(2.0f, 0.0f, -0.30f, p) == PointClass::Obstacle);        // rock side
  assert(classify(2.0f, 0.0f, 0.04f, p) == PointClass::Obstacle);         // at mast height
  assert(classify(2.0f, 0.0f, 0.30f, p) == PointClass::Overhead);         // branch
  // slope tolerance: at 6 m the ground margin grows to 0.24 m
  assert(classify(6.0f, 0.0f, -0.62f, p) == PointClass::Ground);
  assert(classify(6.0f, 0.0f, -0.50f, p) == PointClass::Obstacle);

  // corridor counting (distances from bumper = x - 0.50)
  ZoneCounts c;
  countPoint(1.2f, 0.0f, p, c);   // 0.7 m clear
  assert(c.stop == 1 && c.warn == 1);
  c = ZoneCounts(); countPoint(2.5f, 0.0f, p, c);   // 2.0 m clear
  assert(c.stop == 0 && c.warn == 1);
  c = ZoneCounts(); countPoint(4.2f, 0.0f, p, c);   // beyond warn
  assert(c.stop == 0 && c.warn == 0);
  c = ZoneCounts(); countPoint(1.0f, 0.8f, p, c);   // beside corridor
  assert(c.stop == 0 && c.warn == 0);
  c = ZoneCounts(); countPoint(-1.0f, 0.0f, p, c);  // behind
  assert(c.stop == 0 && c.warn == 0);

  // debounce: 2 on / 3 off
  Debouncer d(2, 3);
  assert(!d.update(true));
  assert(d.update(true));
  assert(d.update(false));
  assert(d.update(false));
  assert(!d.update(false));
  assert(!d.update(true));
  assert(!d.update(false));
  assert(!d.update(true));
  assert(d.update(true));
  assert(d.update(false));
  assert(d.update(true));   // glitch resets the release countdown
  assert(d.update(false));
  assert(d.update(false));
  assert(!d.update(false));

  // obstacle scan: nearest point per bearing bin
  ObstacleScan s;
  s.add(2.0f, 0.0f);
  s.add(1.0f, 0.0f);
  s.add(0.0f, -3.0f);
  const int ahead = 180; // bearing 0 -> bin 180
  assert(std::fabs(s.ranges()[ahead] - 1.0f) < 1e-3);
  assert(std::fabs(s.ranges()[90] - 3.0f) < 1e-3); // bearing -90 deg
  assert(s.ranges()[0] == ObstacleScan::kEmpty);

  // beam height helper: -15 deg beam is 0.40 m above ground at 1.5 m
  assert(std::fabs(beamHeightAt(0.80, 15.0, 1.5) - 0.398) < 0.01);

  std::cout << "test_obstacle_logic: all assertions passed\n";
  return 0;
}
