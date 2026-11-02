#ifndef TRAIL_OBJECT_DETECTION__TRAIL_KEEPER_LOGIC_HPP_
#define TRAIL_OBJECT_DETECTION__TRAIL_KEEPER_LOGIC_HPP_

// ROS-free core of the camera trail-keeper (unit-testable without Gazebo).
// Nav2 drives towards the recorded waypoints but doesn't know where the trail is, so on curves between
// sparse waypoints it cuts across the grass. Each camera frame, this looks at the ground ahead:
//  - if the robot's path ahead is trail (brown), Nav2 is left alone (this includes junctions);
//  - if it is grass, the trail is found to the side and the robot is steered back onto it, choosing the
//    stretch of trail closest to where Nav2 was turning anyway, and slowed down while it gets back on.
// Same flat-ground camera model as water_logic.hpp / trail_logic.hpp: ground d metres ahead of the camera
// and s metres to the LEFT is at row cy + camera_height * f / d, column cx - s * f / d.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "trail_object_detection/trail_logic.hpp"

namespace trail_object_detection
{

struct TrailKeeperParams
{
  TrailParams camera; // camera geometry (camera_ahead_of_sensor = ahead of base_link) and trail colour

  // --- "Am I on the trail?": the robot's own path just ahead (metres from the camera) ---
  double ahead_near = 0.5;
  double ahead_far = 1.2;
  double ahead_half_width = 0.25;
  double on_trail_fraction = 0.5; // at least this share of it brown => on the trail, leave Nav2 alone

  // --- "Where is the trail?": a band further ahead, split into strips across the view ---
  double look_near = 0.9;
  double look_far = 1.6;
  double max_lateral = 1.5;   // strips this far either side
  double bin_width = 0.1;
  double bin_fraction = 0.5;  // a strip at least this brown is trail
  double sample_spacing = 0.1;

  // --- Steering ---
  double min_speed = 0.05;             // only steer while driving forward (not turning on the spot)
  double max_angular = 1.0;            // rad/s
  double off_trail_speed_factor = 0.6; // slow down while getting back onto the trail
};

struct TrailView
{
  bool valid = false;           // false: the camera couldn't see the bands (shouldn't happen with a level camera)
  double ahead_fraction = 1.0;  // brown share of the path just ahead
  std::vector<double> lateral;  // strip centres, metres LEFT of the camera
  std::vector<double> fraction; // brown share of each strip (-1: not in view)
};

struct Steering
{
  bool active = false;     // false: pass Nav2's command through unchanged
  double angular = 0.0;    // rad/s, replaces Nav2's while active
  double speed_factor = 1.0;
  double target = 0.0;     // metres left of the robot the trail was found at (for logging)
};

// Brown share of the ground patch [d0, d1] ahead x [s0, s1] left, or -1 if none of it is in view.
inline double trailFraction(const std::uint8_t *data, int width, int height, int step, int channels, bool bgr,
                            double d0, double d1, double s0, double s1, double spacing, const TrailParams &p)
{
  const double f = (width / 2.0) / std::tan(p.horizontal_fov / 2.0);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const int ri = bgr ? 2 : 0;
  const int bi = bgr ? 0 : 2;
  spacing = std::max(0.01, spacing);
  int samples = 0;
  int trail = 0;
  for (double d = d0; d <= d1 + 1e-9; d += spacing)
  {
    if (d <= 0.0)
    {
      continue;
    }
    const int v = static_cast<int>(std::lround(cy + p.camera_height * f / d - 0.5));
    if (v < 0 || v >= height)
    {
      continue;
    }
    const std::uint8_t *row = data + static_cast<std::ptrdiff_t>(v) * step;
    for (double s = s0; s <= s1 + 1e-9; s += spacing)
    {
      const int u = static_cast<int>(std::lround(cx - s * f / d - 0.5));
      if (u < 0 || u >= width)
      {
        continue;
      }
      const std::uint8_t *px = row + u * channels;
      ++samples;
      trail += isTrailColour(px[ri], px[1], px[bi], p) ? 1 : 0;
    }
  }
  return samples > 0 ? static_cast<double>(trail) / samples : -1.0;
}

// Once per camera frame: how much of the path ahead is trail, and where across the view the trail is.
inline TrailView viewTrail(const std::uint8_t *data, int width, int height, int step, int channels, bool bgr,
                           const TrailKeeperParams &p)
{
  TrailView view;
  const double spacing = p.sample_spacing;
  view.ahead_fraction = trailFraction(data, width, height, step, channels, bgr, p.ahead_near, p.ahead_far,
                                      -p.ahead_half_width, p.ahead_half_width, spacing / 2.0, p.camera);
  const int bins = static_cast<int>(std::lround(2.0 * p.max_lateral / p.bin_width));
  for (int i = 0; i <= bins; ++i)
  {
    const double s = -p.max_lateral + i * p.bin_width;
    view.lateral.push_back(s);
    view.fraction.push_back(trailFraction(data, width, height, step, channels, bgr, p.look_near, p.look_far,
                                          s - p.bin_width / 2.0, s + p.bin_width / 2.0, p.bin_width / 2.0,
                                          p.camera));
  }
  view.valid = view.ahead_fraction >= 0.0;
  return view;
}

// For each Nav2 command (linear v, angular w): keep going, or steer back onto the trail.
inline Steering keepOnTrail(const TrailView &view, double v, double w, const TrailKeeperParams &p)
{
  Steering out;
  if (!view.valid || v < p.min_speed || view.ahead_fraction >= p.on_trail_fraction)
  {
    return out; // on the trail (or turning on the spot / reversing): Nav2 knows best
  }

  // Where Nav2's current arc would be across the look band, so we rejoin the trail on the side it wants
  const double look = (p.look_near + p.look_far) / 2.0;
  const double nav_lateral = std::clamp(w / v * look * look / 2.0, -p.max_lateral, p.max_lateral);

  int best = -1;
  for (std::size_t i = 0; i < view.lateral.size(); ++i)
  {
    if (view.fraction[i] < p.bin_fraction)
    {
      continue;
    }
    if (best < 0 || std::fabs(view.lateral[i] - nav_lateral) < std::fabs(view.lateral[best] - nav_lateral))
    {
      best = static_cast<int>(i);
    }
  }
  if (best < 0)
  {
    return out; // no trail in sight: nothing to steer to
  }

  // Pure pursuit to the trail point, measured from the robot's centre of rotation (base_link)
  const double x = look + p.camera.camera_ahead_of_sensor;
  const double y = view.lateral[best];
  const double curvature = 2.0 * y / (x * x + y * y);
  const double speed = v * p.off_trail_speed_factor;
  out.active = true;
  out.target = y;
  out.speed_factor = p.off_trail_speed_factor;
  out.angular = std::clamp(speed * curvature, -p.max_angular, p.max_angular);
  return out;
}

} // namespace trail_object_detection

#endif
