// Plain-assert unit tests for the ROS-free trail-keeper. Frames are synthesised with the same flat-ground
// camera model: 720x480, 120 deg FOV, camera 0.40 m up; brown trail strips on grass, blue sky.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "trail_object_detection/trail_keeper_logic.hpp"

using namespace trail_object_detection;

struct Rgb { std::uint8_t r, g, b; };

constexpr Rgb kGrass{90, 160, 40};
constexpr Rgb kPath{120, 75, 55};
constexpr Rgb kSky{135, 206, 235};

struct Frame
{
  int w = 720, h = 480, channels = 3;
  std::vector<std::uint8_t> data = std::vector<std::uint8_t>(720 * 480 * 3);
  int step() const { return w * channels; }
};

// Trail strips are (left_min, left_max) metres LEFT of the camera, running straight ahead.
static Frame makeFrame(const TrailParams &p, const std::vector<std::pair<double, double>> &strips)
{
  Frame f;
  const double fpx = (f.w / 2.0) / std::tan(p.horizontal_fov / 2.0);
  for (int v = 0; v < f.h; ++v)
  {
    for (int u = 0; u < f.w; ++u)
    {
      Rgb c = kSky;
      const double below = v + 0.5 - f.h / 2.0;
      if (below > 0)
      {
        const double d = p.camera_height * fpx / below;
        const double left = -(u + 0.5 - f.w / 2.0) * d / fpx;
        c = kGrass;
        for (const auto &[lo, hi] : strips)
        {
          if (left >= lo && left <= hi)
          {
            c = kPath;
          }
        }
      }
      std::uint8_t *px = f.data.data() + v * f.step() + u * 3;
      px[0] = c.r;
      px[1] = c.g;
      px[2] = c.b;
    }
  }
  return f;
}

static Steering steer(const Frame &f, const TrailKeeperParams &p, double v, double w)
{
  const TrailView view = viewTrail(f.data.data(), f.w, f.h, f.step(), f.channels, false, p);
  assert(view.valid);
  return keepOnTrail(view, v, w, p);
}

int main()
{
  const TrailKeeperParams p;

  // On a 1.2 m trail: Nav2 is left alone, whatever it is doing
  const Frame on = makeFrame(p.camera, {{-0.6, 0.6}});
  assert(!steer(on, p, 0.5, 0.0).active);
  assert(!steer(on, p, 0.5, 0.8).active);

  // Cutting a corner: the trail is off to the LEFT => steer left, slowed down
  const Frame left = makeFrame(p.camera, {{0.5, 1.7}});
  const Steering l = steer(left, p, 0.5, 0.0);
  assert(l.active && l.angular > 0.0 && l.target > 0.0);
  assert(l.speed_factor < 1.0);
  // ...and to the RIGHT => steer right
  const Steering r = steer(makeFrame(p.camera, {{-1.7, -0.5}}), p, 0.5, 0.0);
  assert(r.active && r.angular < 0.0 && r.target < 0.0);
  // The further off the trail, the harder the turn
  const Steering far_left = steer(makeFrame(p.camera, {{1.0, 2.0}}), p, 0.5, 0.0);
  assert(far_left.active && far_left.angular > l.angular);

  // Off the trail between two branches (a junction): rejoin the one Nav2 is turning towards
  const Frame fork = makeFrame(p.camera, {{-1.5, -0.6}, {0.6, 1.5}});
  assert(steer(fork, p, 0.5, -0.6).angular < 0.0);
  assert(steer(fork, p, 0.5, 0.6).angular > 0.0);

  // Nothing to steer to, turning on the spot, or reversing: pass through
  assert(!steer(makeFrame(p.camera, {}), p, 0.5, 0.0).active);
  assert(!steer(left, p, 0.0, 1.0).active);
  assert(!steer(left, p, -0.3, 0.0).active);

  // Never steers harder than max_angular
  TrailKeeperParams fast = p;
  fast.max_angular = 0.05;
  assert(std::fabs(steer(left, fast, 0.5, 0.0).angular) <= 0.05 + 1e-9);

  std::cout << "test_trail_keeper_logic: all passed (left " << l.angular << " rad/s towards "
            << l.target << " m)\n";
  return 0;
}
