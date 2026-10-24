// Synthetic replay: ray-casts trail_world-sized obstacles with the real sensor
// model (0.80 m high, 360 azimuth samples, 0.01 m gaussian range noise, min
// range 0.2 m) and runs the perception core on the resulting clouds.
// Sizes come from the measured trail_world models. ROS-free.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "trail_object_detection/obstacle_logic.hpp"

using namespace trail_object_detection;

struct Pt { float x, y, z; };

// ---- scene primitives (sensor frame, ground at z = -0.80) ----
struct Cyl { double cx, cy, r, z0, z1; };
struct Box { double x0, x1, y0, y1, z0, z1; };
struct Scene { std::vector<Cyl> cyls; std::vector<Box> boxes; };

static double hitCyl(const Cyl &c, double dx, double dy, double dz)
{
  const double a = dx * dx + dy * dy;
  if (a < 1e-12) return -1;
  const double b = -2 * (dx * c.cx + dy * c.cy);
  const double cc = c.cx * c.cx + c.cy * c.cy - c.r * c.r;
  const double disc = b * b - 4 * a * cc;
  if (disc < 0) return -1;
  const double t = (-b - std::sqrt(disc)) / (2 * a);
  if (t <= 0) return -1;
  const double z = t * dz;
  return (z >= c.z0 && z <= c.z1) ? t : -1;
}
static double hitBox(const Box &b, double dx, double dy, double dz)
{
  double tmin = 0, tmax = 1e9;
  const double o[3] = {0, 0, 0}, d[3] = {dx, dy, dz};
  const double lo[3] = {b.x0, b.y0, b.z0}, hi[3] = {b.x1, b.y1, b.z1};
  for (int i = 0; i < 3; ++i)
  {
    if (std::fabs(d[i]) < 1e-12)
    {
      if (o[i] < lo[i] || o[i] > hi[i]) return -1;
    }
    else
    {
      double t1 = (lo[i] - o[i]) / d[i], t2 = (hi[i] - o[i]) / d[i];
      if (t1 > t2) std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) return -1;
    }
  }
  return tmin > 0 ? tmin : -1;
}

static std::vector<double> ringsOld16() { std::vector<double> r; for (int i = 0; i < 16; ++i) r.push_back(-15 + 2.0 * i); return r; }
static double g_step = 2.5; // vertical spacing of the new ring set (deg)
static std::vector<double> ringsNew17() { std::vector<double> r; const int n = int(std::lround(80.0 / g_step)) + 1; for (int i = 0; i < n; ++i) r.push_back(-40 + g_step * i); return r; }

static std::mt19937 rng(42);

static std::vector<Pt> scan(const Scene &s, const std::vector<double> &rings_deg)
{
  Scene sc = s;
  // Robot's own body below the sensor (top plate, chassis, bumpers) is rendered by gpu_lidar too.
  sc.boxes.push_back({-0.49, 0.49, -0.285, 0.285, -0.68, -0.43});
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<Pt> pts;
  for (double el : rings_deg)
  {
    const double e = el * kPi / 180.0;
    for (int k = 0; k < 360; ++k)
    {
      const double az = k * kPi / 180.0;
      const double dx = std::cos(e) * std::cos(az), dy = std::cos(e) * std::sin(az), dz = std::sin(e);
      double t = 40;
      if (dz < 0) t = std::min(t, -0.80 / dz);
      for (auto &c : sc.cyls) { double h = hitCyl(c, dx, dy, dz); if (h > 0) t = std::min(t, h); }
      for (auto &b : sc.boxes) { double h = hitBox(b, dx, dy, dz); if (h > 0) t = std::min(t, h); }
      if (t >= 40 || t < 0.2) continue;
      t += noise(rng);
      pts.push_back({float(t * dx), float(t * dy), float(t * dz)});
    }
  }
  return pts;
}

struct Result { size_t warn, stop; bool stopBool, warnBool; };

// Run n identical frames through the new logic with the default debounce (2 on / 3 off).
static Result runNew(const std::vector<Pt> &cloud, const Params &p = Params())
{
  Debouncer wd(2, 3), sd(2, 3);
  ZoneCounts c;
  for (auto &pt : cloud)
    if (classify(pt.x, pt.y, pt.z, p) == PointClass::Obstacle) countPoint(pt.x, pt.y, p, c);
  bool w = false, st = false;
  for (int i = 0; i < 3; ++i) { w = wd.update(c.warn >= 3); st = sd.update(c.stop >= 3); }
  return {c.warn, c.stop, st, w};
}

// The OLD groupmate rule (ground_filter.cpp in the zip): z in [-0.72, 1.5], x>0,
// range <= 0.45 from the sensor, |y|<=0.5, a single point stops, no debounce.
static bool oldStop(const std::vector<Pt> &cloud)
{
  size_t n = 0;
  for (auto &p : cloud)
    if (std::isfinite(p.x) && p.z >= -0.72f && p.z <= 1.5f && p.x > 0 && std::hypot(p.x, p.y) <= 0.45 && std::fabs(p.y) <= 0.5) ++n;
  return n >= 1;
}

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("  FAIL: %s\n", msg); ++failures; } } while (0)

struct Case { const char *name; Scene scene; bool expectStop; bool expectWarn; };

int main(int argc, char **argv)
{
  if (argc > 1) g_step = std::atof(argv[1]);
  const auto n17 = ringsNew17();
  const double G = -0.80;
  std::vector<Case> cases;
  cases.push_back({"empty trail (ground only)", {}, false, false});
  cases.push_back({"grey rock 1.1 m tall, 0.8 m from bumper", {{{1.30 + 0.4, 0, 0.4, G, G + 1.1}}, {}}, true, true});
  cases.push_back({"grey rock 1.1 m tall, 2.5 m from bumper", {{{3.00 + 0.4, 0, 0.4, G, G + 1.1}}, {}}, false, true});
  cases.push_back({"rock 1.8 m, beside path (edge 0.6 m off centre)", {{{1.5, 1.0, 0.4, G, G + 1.8}}, {}}, false, false});
  cases.push_back({"tree trunk 3 m tall, 0.7 m from bumper", {{{1.2 + 0.15, 0, 0.15, G, G + 3.0}}, {}}, true, true});
  cases.push_back({"large fence 0.24 m tall, 2 m wide, 0.9 m from bumper", {{}, {{1.40, 1.46, -1.0, 1.0, G, G + 0.24}}}, true, true});
  cases.push_back({"mushroom 0.15 m tall, 0.8 m from bumper", {{{1.30 + 0.08, 0, 0.08, G, G + 0.15}}, {}}, true, true});
  cases.push_back({"flower 0.20 m tall, 0.9 m from bumper", {{{1.40 + 0.05, 0, 0.05, G, G + 0.20}}, {}}, true, true});
  cases.push_back({"pebble 3 cm tall (drivable)", {{{1.2, 0, 0.05, G, G + 0.03}}, {}}, false, false});
  cases.push_back({"OVERHEAD branch ahead (1.05-1.25 m high), spans path", {{}, {{0.6, 3.0, -1.0, 1.0, 0.25, 0.45}}}, false, false});
  cases.push_back({"branch underside 0.90 m high, robot driving under it", {{}, {{-0.4, 3.0, -1.0, 1.0, 0.10, 0.30}}}, false, false});
  cases.push_back({"branch underside 0.65 m high ahead (would hit robot)", {{}, {{1.3, 1.5, -0.8, 0.8, -0.15, 0.0}}}, true, true});

  std::printf("%-58s %-12s %-12s\n", "scenario", "OLD filter", "NEW filter");
  for (auto &c : cases)
  {
    const auto cloud17 = scan(c.scene, n17);
    const auto cloud16 = scan(c.scene, ringsOld16());
    const bool oldS = oldStop(cloud16);
    const Result r = runNew(cloud17);
    std::printf("%-58s %-12s %-12s (stop pts %zu, warn pts %zu)\n", c.name, oldS ? "STOP" : "go",
                r.stopBool ? "STOP" : "go", r.stop, r.warn);
    CHECK(r.stopBool == c.expectStop, c.name);
    CHECK(r.warnBool == c.expectWarn, c.name);
  }

  // ---- Approach sweeps: first distance (from bumper) at which a low object produces a stop-zone/warn detection ----
  std::printf("\nApproach check (object front distance from bumper -> detected?)\n");
  struct Obj { const char *name; double r, h; };
  const Obj objs[] = {{"mushroom 0.12 m", 0.06, 0.12}, {"mushroom 0.15 m", 0.08, 0.15}, {"flower 0.20 m", 0.05, 0.20}, {"fence 0.25 m (2 m wide)", 0, 0.25}};
  for (auto &o : objs)
  {
    std::string seen;
    double firstSeen = -1, gapStart = -1, worstGap = 0;
    for (double ahead = 3.0; ahead >= 0.05; ahead -= 0.05)
    {
      Scene s;
      const double front = 0.50 + ahead;
      if (o.r > 0) s.cyls.push_back({front + o.r, 0, o.r, G, G + o.h});
      else s.boxes.push_back({front, front + 0.06, -1.0, 1.0, G, G + o.h});
      Params p; p.warn_distance = 10; p.stop_distance = 10; // count anywhere in view
      const Result r = runNew(scan(s, n17), p);
      const bool seenNow = r.warn >= 3;
      if (seenNow && firstSeen < 0) firstSeen = ahead;
      if (firstSeen >= 0 && !seenNow) { if (gapStart < 0) gapStart = ahead; worstGap = std::max(worstGap, gapStart - ahead); }
      else gapStart = -1;
    }
    std::printf("  %-26s first seen at %.2f m from bumper, longest blind gap after that %.2f m\n", o.name, firstSeen, worstGap);
    CHECK(firstSeen >= 1.3, "low object must be seen before the 1.0 m stop line");
    CHECK(worstGap <= 0.45, "blind gap shorter than debounce hold");
  }

  std::printf("\n%s (%d failures)\n", failures ? "TESTS FAILED" : "ALL SCENARIOS PASSED", failures);
  return failures ? 1 : 0;
}
