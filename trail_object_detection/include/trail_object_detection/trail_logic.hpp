#ifndef TRAIL_OBJECT_DETECTION__TRAIL_LOGIC_HPP_
#define TRAIL_OBJECT_DETECTION__TRAIL_LOGIC_HPP_

// ROS-free camera check: is a LiDAR obstacle standing on the (brown) trail, or beside it on the grass?
// Uses the same flat-ground camera model as water_logic.hpp: a level, forward-facing pinhole camera,
// where ground d metres ahead of the camera is image row v = cy + camera_height * f / d, and ground
// s metres to the LEFT is column u = cx - s * f / d.
//
// The obstacle itself can't be used (tree bark is brown too), so the ground patch just IN FRONT of the
// obstacle point is checked instead. Anything standing on the ground at distance d only covers rows
// above its base row, so the rows below it always show ground, never the obstacle.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <unordered_map>

namespace trail_object_detection
{

struct TrailParams
{
  // --- Camera (husky.urdf.xacro / husky.gazebo.xacro), see WaterParams ---
  double camera_height = 0.40;
  double horizontal_fov = 2.0944;
  double camera_ahead_of_sensor = 0.455; // camera_joint x 0.45 + rgb frame 0.005; the LiDAR is at base_link x = 0

  // --- Colour (HSV): trail_world's path texture is hue ~15-20 deg, grass ~90, pond ~205 ---
  double hue_min = 0.0; // deg
  double hue_max = 40.0;
  double sat_min = 0.25;
  double val_min = 0.15;

  // --- Ground patch checked in front of each obstacle point (metres, along / across the view) ---
  double patch_near = 0.35;      // patch runs from (d - patch_near) ...
  double patch_far = 0.05;       // ... to (d - patch_far) ahead of the camera, d = obstacle distance
  double patch_half_width = 0.15;
  double sample_spacing = 0.05;
  int min_samples = 6;              // fewer visible samples than this => can't tell
  double min_trail_fraction = 0.25; // this share of the patch must be trail colour for "on trail"
};

enum class TrailCheck
{
  OnTrail,
  OffTrail,
  Unknown // patch not in view (too close, or off the image edge)
};

inline bool isTrailColour(std::uint8_t r, std::uint8_t g, std::uint8_t b, const TrailParams &p)
{
  const int mx = std::max({r, g, b});
  const int mn = std::min({r, g, b});
  if (mx == 0 || mx - mn == 0)
  {
    return false; // black / grey has no hue
  }
  const double delta = mx - mn;
  if (mx / 255.0 < p.val_min || delta / mx < p.sat_min)
  {
    return false;
  }
  double hue;
  if (mx == r)
  {
    hue = 60.0 * std::fmod((g - b) / delta + 6.0, 6.0);
  }
  else if (mx == g)
  {
    hue = 60.0 * ((b - r) / delta + 2.0);
  }
  else
  {
    hue = 60.0 * ((r - g) / delta + 4.0);
  }
  return hue >= p.hue_min && hue <= p.hue_max;
}

// Check the ground in front of an obstacle point at (x, y) in the LiDAR frame (x forward, y left).
// `data` is packed 8-bit RGB(A) or BGR(A); `channels` is 3 or 4.
// on_sample(u, v, is_trail) is called for every patch sample in view (debug overlay).
template <typename OnSample>
TrailCheck checkOnTrail(const std::uint8_t *data, int width, int height, int step, int channels,
                        bool bgr, double x, double y, const TrailParams &p, OnSample on_sample)
{
  const double f = (width / 2.0) / std::tan(p.horizontal_fov / 2.0);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const int ri = bgr ? 2 : 0;
  const int bi = bgr ? 0 : 2;
  const double d_obstacle = x - p.camera_ahead_of_sensor;
  const double spacing = std::max(0.01, p.sample_spacing);

  int samples = 0;
  int trail = 0;
  for (double d = d_obstacle - p.patch_near; d <= d_obstacle - p.patch_far + 1e-9; d += spacing)
  {
    if (d <= 0.0)
    {
      continue;
    }
    const int v = static_cast<int>(std::lround(cy + p.camera_height * f / d - 0.5));
    if (v < 0 || v >= height)
    {
      continue; // closer than the bottom of the image
    }
    for (double s = -p.patch_half_width; s <= p.patch_half_width + 1e-9; s += spacing)
    {
      const int u = static_cast<int>(std::lround(cx - (y + s) * f / d - 0.5));
      if (u < 0 || u >= width)
      {
        continue;
      }
      const std::uint8_t *px = data + static_cast<std::ptrdiff_t>(v) * step + u * channels;
      const bool is_trail = isTrailColour(px[ri], px[1], px[bi], p);
      on_sample(u, v, is_trail);
      ++samples;
      trail += is_trail ? 1 : 0;
    }
  }
  if (samples < p.min_samples)
  {
    return TrailCheck::Unknown;
  }
  return trail >= p.min_trail_fraction * samples ? TrailCheck::OnTrail : TrailCheck::OffTrail;
}

inline TrailCheck checkOnTrail(const std::uint8_t *data, int width, int height, int step, int channels,
                               bool bgr, double x, double y, const TrailParams &p)
{
  return checkOnTrail(data, width, height, step, channels, bgr, x, y, p, [](int, int, bool) {});
}

// Remembers the camera's verdicts by position in a fixed frame (odom), so an obstacle judged to be beside
// the trail stays ignored once the robot is too close for the camera to see the ground in front of it
// (or between camera frames). A nearby ON-trail verdict always wins, so a rock on the trail next to a
// tree beside it is never ignored.
class TrailMemory
{
public:
  explicit TrailMemory(double cell_size = 0.2, double max_age = 120.0)
      : cell_(cell_size), max_age_(max_age) {}

  void mark(double x, double y, bool on_trail, double now)
  {
    cells_[key(cellOf(x), cellOf(y))] = {on_trail, now};
  }

  // True if this spot (or a neighbouring cell) was judged beside the trail recently and nothing nearby on it.
  bool offTrail(double x, double y, double now) const
  {
    const long cx = cellOf(x), cy = cellOf(y);
    bool off = false;
    for (long dx = -1; dx <= 1; ++dx)
    {
      for (long dy = -1; dy <= 1; ++dy)
      {
        const auto it = cells_.find(key(cx + dx, cy + dy));
        if (it == cells_.end() || now - it->second.time > max_age_)
        {
          continue;
        }
        if (it->second.on_trail)
        {
          return false;
        }
        off = true;
      }
    }
    return off;
  }

  void prune(double now)
  {
    for (auto it = cells_.begin(); it != cells_.end();)
    {
      it = (now - it->second.time > max_age_) ? cells_.erase(it) : std::next(it);
    }
  }

  std::size_t size() const { return cells_.size(); }

private:
  struct Verdict
  {
    bool on_trail;
    double time;
  };
  long cellOf(double v) const { return static_cast<long>(std::floor(v / cell_)); }
  static long long key(long x, long y) { return (static_cast<long long>(x) << 32) ^ (y & 0xffffffffLL); }

  double cell_;
  double max_age_;
  std::unordered_map<long long, Verdict> cells_;
};

} // namespace trail_object_detection

#endif
