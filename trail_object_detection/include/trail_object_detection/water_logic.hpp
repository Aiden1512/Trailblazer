#ifndef TRAIL_OBJECT_DETECTION__WATER_LOGIC_HPP_
#define TRAIL_OBJECT_DETECTION__WATER_LOGIC_HPP_

// ROS-free core of the camera water guard (unit-testable without Gazebo).
// Camera: level, forward-facing pinhole (husky1_camera_rgb_frame) over flat ground.
// A ground pixel at image row v below the horizon row cy is
//   d = camera_height * f / (v - cy)      metres ahead of the camera,
// and column u is (u - cx) * d / f metres to the side. Rows at or above the
// horizon are never looked at, so blue sky can't trigger the guard.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace trail_object_detection
{

struct WaterParams
{
  // --- Camera (husky.urdf.xacro / husky.gazebo.xacro) ---
  double camera_height = 0.40;    // base_link 0.13 m above ground + camera 0.25 m + rgb frame 0.013 m
  double horizontal_fov = 2.0944; // 120 deg; nearest visible ground is then ~0.34 m ahead

  // --- Colour (HSV): trail_world's pond texture is hue ~200-210 deg, grass ~90, path ~15 ---
  double hue_min = 185.0; // deg
  double hue_max = 250.0;
  double sat_min = 0.45;
  double val_min = 0.20;

  // --- Corridor (distances measured from the CAMERA, which sits at the front bumper) ---
  double corridor_half_width = 0.45; // Husky is 0.67 m wide
  double stop_distance = 0.9;        // forward driving blocked inside this
  double release_distance = 1.1;     // ...until the water is further than this again
  double slow_distance = 1.8;        // forward speed is scaled down inside this
  double min_speed_factor = 0.3;     // forward speed factor just outside stop_distance
  double max_range = 3.0;            // rows further away than this are ignored

  int min_pixels_per_row = 4; // water samples a row needs (after stride) to count
  int pixel_stride = 2;       // sample every Nth row and column
};

inline bool isWaterColour(std::uint8_t r, std::uint8_t g, std::uint8_t b, const WaterParams &p)
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

struct WaterScan
{
  double nearest = std::numeric_limits<double>::infinity(); // metres ahead of the camera
  int corridor_pixels = 0;                                  // water samples inside the corridor
};

// Find the nearest row of water inside the robot's corridor.
// `data` is packed 8-bit RGB(A) or BGR(A); `channels` is 3 or 4.
// on_water(u, v, in_corridor) is called for every water sample (debug overlay).
template <typename OnWater>
WaterScan scanForWater(const std::uint8_t *data, int width, int height, int step,
                       int channels, bool bgr, const WaterParams &p, OnWater on_water)
{
  WaterScan scan;
  const double f = (width / 2.0) / std::tan(p.horizontal_fov / 2.0);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const int stride = std::max(1, p.pixel_stride);
  const int ri = bgr ? 2 : 0;
  const int bi = bgr ? 0 : 2;

  // Bottom row first: nearest ground first, so the first row that qualifies is the nearest.
  for (int v = height - 1; v > cy; v -= stride)
  {
    const double d = p.camera_height * f / (v + 0.5 - cy);
    if (d > p.max_range)
    {
      break; // every row above is further still
    }
    const double half_px = p.corridor_half_width * f / d;
    const int u0 = std::max(0, static_cast<int>(std::floor(cx - half_px)));
    const int u1 = std::min(width - 1, static_cast<int>(std::ceil(cx + half_px)));

    const std::uint8_t *row = data + static_cast<std::ptrdiff_t>(v) * step;
    int count = 0;
    for (int u = 0; u < width; u += stride)
    {
      const std::uint8_t *px = row + u * channels;
      if (!isWaterColour(px[ri], px[1], px[bi], p))
      {
        continue;
      }
      const bool in_corridor = u >= u0 && u <= u1;
      on_water(u, v, in_corridor);
      if (in_corridor)
      {
        ++count;
      }
    }
    scan.corridor_pixels += count;
    if (count >= p.min_pixels_per_row && d < scan.nearest)
    {
      scan.nearest = d;
    }
  }
  return scan;
}

inline WaterScan scanForWater(const std::uint8_t *data, int width, int height, int step,
                              int channels, bool bgr, const WaterParams &p)
{
  return scanForWater(data, width, height, step, channels, bgr, p, [](int, int, bool) {});
}

// Image row that sees ground `d` metres ahead (for drawing the zones), or -1 if off-image.
inline int rowForDistance(double d, int width, int height, const WaterParams &p)
{
  const double f = (width / 2.0) / std::tan(p.horizontal_fov / 2.0);
  const int v = static_cast<int>(std::lround(height / 2.0 + p.camera_height * f / d - 0.5));
  return (v >= 0 && v < height) ? v : -1;
}

// Turns the nearest-water distance of each camera frame into a forward speed
// factor. Latches the stop until the water is beyond release_distance, so it
// can't flicker on and off right at the edge.
class WaterGate
{
public:
  explicit WaterGate(const WaterParams &p = WaterParams()) : p_(p) {}

  double update(double nearest)
  {
    if (stopped_)
    {
      stopped_ = !(nearest > p_.release_distance);
    }
    else
    {
      stopped_ = nearest <= p_.stop_distance;
    }
    if (stopped_)
    {
      factor_ = 0.0;
    }
    else if (nearest >= p_.slow_distance)
    {
      factor_ = 1.0;
    }
    else
    {
      const double t = (nearest - p_.stop_distance) / (p_.slow_distance - p_.stop_distance);
      factor_ = p_.min_speed_factor + (1.0 - p_.min_speed_factor) * std::clamp(t, 0.0, 1.0);
    }
    return factor_;
  }

  // Camera lost: block forward driving until a frame shows it is clear again.
  void blind()
  {
    stopped_ = true;
    factor_ = 0.0;
  }

  bool stopped() const { return stopped_; }
  bool slowed() const { return !stopped_ && factor_ < 1.0; }
  double factor() const { return factor_; }

private:
  WaterParams p_;
  bool stopped_ = false;
  double factor_ = 1.0;
};

// Reversing and turning are always allowed; only forward speed is limited.
inline double limitForward(double linear_x, double factor)
{
  return linear_x > 0.0 ? linear_x * factor : linear_x;
}

} // namespace trail_object_detection

#endif
