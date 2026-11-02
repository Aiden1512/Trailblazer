// Plain-assert unit tests for the ROS-free trail check. Frames are synthesised with the same
// flat-ground camera model: 720x480, 120 deg FOV, camera 0.40 m up and 0.455 m ahead of the LiDAR,
// a brown trail strip on grass, blue sky above the horizon.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "trail_object_detection/trail_logic.hpp"

using namespace trail_object_detection;

struct Rgb { std::uint8_t r, g, b; };

constexpr Rgb kGrass{90, 160, 40};
constexpr Rgb kPath{120, 75, 55};    // trail_world path texture
constexpr Rgb kPathTex{133, 82, 53}; // sampled straight from Floor_Texture.png
constexpr Rgb kSky{135, 206, 235};

struct Frame
{
  int w = 720, h = 480, channels = 3;
  std::vector<std::uint8_t> data = std::vector<std::uint8_t>(720 * 480 * 3);
  int step() const { return w * channels; }
};

// Trail is the strip y_min..y_max metres to the LEFT of the camera (LiDAR y convention), all the way ahead.
static Frame makeFrame(const TrailParams &p, double y_min, double y_max, Rgb path = kPath)
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
        c = (left >= y_min && left <= y_max) ? path : kGrass;
      }
      std::uint8_t *px = f.data.data() + v * f.step() + u * 3;
      px[0] = c.r;
      px[1] = c.g;
      px[2] = c.b;
    }
  }
  return f;
}

static TrailCheck check(const Frame &f, const TrailParams &p, double x, double y)
{
  return checkOnTrail(f.data.data(), f.w, f.h, f.step(), f.channels, false, x, y, p);
}

int main()
{
  const TrailParams p;

  // Colours
  assert(isTrailColour(kPath.r, kPath.g, kPath.b, p));
  assert(isTrailColour(kPathTex.r, kPathTex.g, kPathTex.b, p));
  assert(!isTrailColour(kGrass.r, kGrass.g, kGrass.b, p));
  assert(!isTrailColour(85, 139, 42, p));  // texture grass
  assert(!isTrailColour(0, 120, 200, p));  // pond
  assert(!isTrailColour(128, 128, 128, p)); // grey rock face has no hue

  // Straight trail 1.2 m wide, robot in the middle
  const Frame straight = makeFrame(p, -0.6, 0.6);
  assert(check(straight, p, 1.5, 0.0) == TrailCheck::OnTrail);   // rock in the middle of the trail
  assert(check(straight, p, 1.5, 0.3) == TrailCheck::OnTrail);   // still on it, near the edge
  assert(check(straight, p, 1.5, 1.2) == TrailCheck::OffTrail);  // tree on the grass to the left
  assert(check(straight, p, 1.5, -1.2) == TrailCheck::OffTrail); // ...and to the right
  assert(check(straight, p, 3.5, 1.5) == TrailCheck::OffTrail);  // far end of the warn zone

  // Trail curving away to the left: only the left side is trail, so left/right must not be mixed up
  const Frame left_only = makeFrame(p, 0.2, 2.0);
  assert(check(left_only, p, 1.5, 0.8) == TrailCheck::OnTrail);
  assert(check(left_only, p, 1.5, -0.4) == TrailCheck::OffTrail);

  // BGR frames give the same answer
  Frame bgr = straight;
  for (std::size_t i = 0; i + 2 < bgr.data.size(); i += 3)
  {
    std::swap(bgr.data[i], bgr.data[i + 2]);
  }
  assert(checkOnTrail(bgr.data.data(), bgr.w, bgr.h, bgr.step(), 3, true, 1.5, 0.0, p) == TrailCheck::OnTrail);
  assert(checkOnTrail(bgr.data.data(), bgr.w, bgr.h, bgr.step(), 3, true, 1.5, 1.2, p) == TrailCheck::OffTrail);

  // Too close for the camera to see the ground in front of it: can't tell, so LiDAR decides
  assert(check(straight, p, 0.6, 0.0) == TrailCheck::Unknown);
  assert(check(straight, p, 0.3, 0.0) == TrailCheck::Unknown);

  // Every sample the callback sees is inside the image
  int samples = 0;
  checkOnTrail(straight.data.data(), straight.w, straight.h, straight.step(), 3, false, 1.5, 0.0, p,
               [&](int u, int v, bool) {
                 assert(u >= 0 && u < straight.w && v >= 0 && v < straight.h);
                 ++samples;
               });
  assert(samples >= p.min_samples);

  // Memory: a tree judged beside the trail from afar stays ignored when it's too close to judge again
  TrailMemory memory(0.2, 120.0);
  memory.mark(10.0, 5.0, false, 0.0);              // tree trunk, odom frame
  assert(memory.offTrail(10.0, 5.0, 1.0));
  assert(memory.offTrail(10.15, 5.1, 1.0));        // the trunk seen from another angle, next cell
  assert(!memory.offTrail(11.0, 5.0, 1.0));        // somewhere else entirely: unknown => not ignored
  assert(!memory.offTrail(10.0, 5.0, 200.0));      // forgotten after max_age
  memory.mark(10.2, 5.0, true, 2.0);               // a rock ON the trail right next to it
  assert(!memory.offTrail(10.0, 5.0, 3.0));        // on-trail verdict nearby wins: never ignore the rock
  memory.prune(500.0);
  assert(memory.size() == 0);
  memory.mark(-3.0, -7.5, false, 0.0);             // negative coordinates hash correctly too
  assert(memory.offTrail(-3.0, -7.5, 1.0));
  assert(!memory.offTrail(3.0, 7.5, 1.0));

  std::cout << "test_trail_logic: all passed (" << samples << " samples per patch)\n";
  return 0;
}
