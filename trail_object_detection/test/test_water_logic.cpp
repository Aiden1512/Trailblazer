// Plain-assert unit tests for the ROS-free water guard core. Frames are
// synthesised with the same flat-ground camera model: 720x480, 120 deg FOV,
// camera 0.40 m up, grass everywhere, blue sky above the horizon.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "trail_object_detection/water_logic.hpp"

using namespace trail_object_detection;

struct Rgb { std::uint8_t r, g, b; };

constexpr Rgb kGrass{90, 160, 40};
constexpr Rgb kPath{120, 75, 55};
constexpr Rgb kPond{0, 120, 200};
constexpr Rgb kSky{135, 206, 235};

struct Frame
{
  int w = 720, h = 480, channels = 3;
  std::vector<std::uint8_t> data = std::vector<std::uint8_t>(720 * 480 * 3);
  int step() const { return w * channels; }
};

// Ground water is a rectangle [x0, x1] ahead x [y0, y1] sideways (metres, camera frame).
static Frame makeFrame(const WaterParams &p, double x0, double x1, double y0, double y1,
                       Rgb ground = kGrass, Rgb sky = kSky)
{
  Frame f;
  const double fpx = (f.w / 2.0) / std::tan(p.horizontal_fov / 2.0);
  for (int v = 0; v < f.h; ++v)
  {
    for (int u = 0; u < f.w; ++u)
    {
      Rgb c = sky;
      const double below = v + 0.5 - f.h / 2.0;
      if (below > 0)
      {
        const double d = p.camera_height * fpx / below;
        const double side = (u + 0.5 - f.w / 2.0) * d / fpx;
        c = (d >= x0 && d <= x1 && side >= y0 && side <= y1) ? kPond : ground;
      }
      std::uint8_t *px = f.data.data() + v * f.step() + u * 3;
      px[0] = c.r;
      px[1] = c.g;
      px[2] = c.b;
    }
  }
  return f;
}

static WaterScan scan(const Frame &f, const WaterParams &p)
{
  return scanForWater(f.data.data(), f.w, f.h, f.step(), f.channels, false, p);
}

int main()
{
  const WaterParams p;

  // colour classification
  assert(isWaterColour(kPond.r, kPond.g, kPond.b, p));
  assert(isWaterColour(20, 150, 210, p));            // lighter pond facet
  assert(isWaterColour(0, 70, 120, p));              // pond in shadow
  assert(!isWaterColour(kGrass.r, kGrass.g, kGrass.b, p));
  assert(!isWaterColour(kPath.r, kPath.g, kPath.b, p));
  assert(!isWaterColour(128, 128, 128, p));          // grey
  assert(!isWaterColour(0, 0, 0, p));
  assert(!isWaterColour(10, 20, 35, p));             // too dark
  assert(!isWaterColour(150, 170, 200, p));          // washed-out blue-grey

  // no water: blue sky alone must not trigger
  WaterScan s = scan(makeFrame(p, 99, 99, 0, 0), p);
  assert(!std::isfinite(s.nearest) && s.corridor_pixels == 0);

  // pond 1.5 m ahead, straddling the path: nearest is its near edge
  // (sampled rows are ~5 cm of ground apart at 1.5 m, ~2 cm at the stop distance)
  s = scan(makeFrame(p, 1.5, 10, -2, 2), p);
  assert(std::fabs(s.nearest - 1.5) < 0.08);

  // pond 0.6 m ahead
  s = scan(makeFrame(p, 0.6, 10, -2, 2), p);
  assert(std::fabs(s.nearest - 0.6) < 0.03);

  // robot already at the edge: water fills the bottom row (~0.34 m)
  s = scan(makeFrame(p, 0.0, 10, -2, 2), p);
  assert(s.nearest < 0.40);

  // pond beside the corridor (0.8 m to the left) is seen but ignored
  s = scan(makeFrame(p, 0.5, 3, 0.8, 3), p);
  assert(!std::isfinite(s.nearest) && s.corridor_pixels == 0);

  // corridor edge: water just inside it counts
  s = scan(makeFrame(p, 0.8, 3, 0.35, 3), p);
  assert(std::isfinite(s.nearest));

  // beyond max_range is ignored
  s = scan(makeFrame(p, 4.0, 10, -2, 2), p);
  assert(!std::isfinite(s.nearest));

  // BGR input gives the same answer
  {
    Frame f = makeFrame(p, 1.0, 10, -2, 2);
    for (std::size_t i = 0; i < f.data.size(); i += 3)
    {
      std::swap(f.data[i], f.data[i + 2]);
    }
    s = scanForWater(f.data.data(), f.w, f.h, f.step(), 3, true, p);
    assert(std::fabs(s.nearest - 1.0) < 0.05);
  }

  // rowForDistance agrees with the scan geometry
  assert(rowForDistance(1.0, 720, 480, p) > 240);
  assert(rowForDistance(1.0, 720, 480, p) > rowForDistance(2.0, 720, 480, p));
  assert(rowForDistance(0.1, 720, 480, p) == -1);    // under the camera's view

  // gate: approach, stop, hysteresis, back away
  const double inf = std::numeric_limits<double>::infinity();
  WaterGate g(p);
  assert(g.update(inf) == 1.0 && !g.stopped() && !g.slowed());
  assert(g.update(2.5) == 1.0);
  double f1 = g.update(1.6);
  double f2 = g.update(1.2);
  assert(f1 < 1.0 && f2 < f1 && f2 >= p.min_speed_factor && g.slowed());
  assert(g.update(0.85) == 0.0 && g.stopped());
  assert(g.update(1.0) == 0.0 && g.stopped());       // inside release distance: still latched
  assert(g.update(1.2) > 0.0 && !g.stopped());       // backed off past release
  assert(g.update(0.4) == 0.0 && g.stopped());
  assert(g.update(inf) == 1.0 && !g.stopped());      // turned away: clear

  // blind: blocked until a frame says otherwise
  g.blind();
  assert(g.stopped() && g.factor() == 0.0);
  assert(g.update(0.95) == 0.0);                     // camera back but water close
  assert(g.update(inf) == 1.0);

  // only forward speed is limited
  assert(limitForward(0.5, 0.0) == 0.0);
  assert(limitForward(0.5, 0.5) == 0.25);
  assert(limitForward(-0.5, 0.0) == -0.5);
  assert(limitForward(0.0, 0.0) == 0.0);

  std::cout << "test_water_logic: all passed\n";
  return 0;
}
