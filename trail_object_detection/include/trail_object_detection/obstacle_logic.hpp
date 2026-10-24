#ifndef TRAIL_OBJECT_DETECTION__OBSTACLE_LOGIC_HPP_
#define TRAIL_OBJECT_DETECTION__OBSTACLE_LOGIC_HPP_

// ROS-free perception core (unit-testable without Gazebo).
// Frame: LiDAR frame (husky1_base_scan): x forward, y left, z up, origin at the sensor.
// The sensor is level, 0.80 m above flat ground (ground reads z = -0.80).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace trail_object_detection
{

constexpr double kPi = 3.14159265358979323846;

struct Params
{
  // --- Ground model ---
  double sensor_height = 0.80; // sensor above ground [m]
  double ground_margin = 0.08; // anything lower than this above ground is "ground" [m]
  double slope_tolerance = 0.04; // extra margin per metre of range (about 2.3 deg of pitch/slope)

  // --- Robot body (own hits are ignored) ---
  double body_front = 0.52;  // Husky front is 0.49 m ahead of the sensor
  double body_rear = 0.52;
  double body_half_width = 0.34; // Husky is 0.57 m wide

  // --- Overhead rejection ---
  // Robot top is the LiDAR mast, about +0.02 m above the sensor. Points higher
  // than this pass over the robot (branches) and are never obstacles.
  double overhead_z = 0.06; // 4 cm clearance over the mast; TUNE to your tree models

  // --- Corridor (distances measured from the FRONT BUMPER) ---
  double sensor_to_front = 0.50;
  double corridor_half_width = 0.45;
  double stop_distance = 1.0;
  double warn_distance = 3.5;

  double max_range = 8.0; // ignore returns further than this
};

enum class PointClass
{
  Invalid,  // NaN / inf / out of range
  Self,     // robot's own body
  Ground,   // flat ground (or too low to matter)
  Overhead, // above the robot: branches, canopy
  Obstacle  // something the robot body could hit
};

// Extra ground margin that grows with range, so a small pitch or slope does
// not turn far ground into "obstacles".
inline double groundMargin(double range, const Params &p)
{
  return std::max(p.ground_margin, p.slope_tolerance * range);
}

inline PointClass classify(float fx, float fy, float fz, const Params &p)
{
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz))
  {
    return PointClass::Invalid;
  }
  const double x = fx, y = fy, z = fz;
  const double r = std::hypot(x, y);
  if (r > p.max_range)
  {
    return PointClass::Invalid;
  }
  if (x < p.body_front && x > -p.body_rear && std::fabs(y) < p.body_half_width)
  {
    return PointClass::Self;
  }
  if (z > p.overhead_z)
  {
    return PointClass::Overhead;
  }
  if (z < -p.sensor_height + groundMargin(r, p))
  {
    return PointClass::Ground;
  }
  return PointClass::Obstacle;
}

struct ZoneCounts
{
  std::size_t warn = 0;
  std::size_t stop = 0;
};

// Count an Obstacle point against the warn/stop zones in front of the bumper.
inline void countPoint(float x, float y, const Params &p, ZoneCounts &c)
{
  if (x <= 0.0f || std::fabs(static_cast<double>(y)) > p.corridor_half_width)
  {
    return;
  }
  const double ahead = static_cast<double>(x) - p.sensor_to_front;
  if (ahead <= p.warn_distance)
  {
    ++c.warn;
  }
  if (ahead <= p.stop_distance)
  {
    ++c.stop;
  }
}

// ON after `on_frames` consecutive positives, OFF after `off_frames` consecutive
// negatives. mission_manager cancels/re-sends its Nav2 goal on every change,
// so flicker here would make the robot stutter.
class Debouncer
{
public:
  Debouncer(int on_frames = 1, int off_frames = 1)
      : on_frames_(std::max(1, on_frames)), off_frames_(std::max(1, off_frames)) {}

  bool update(bool raw)
  {
    if (raw)
    {
      ++on_count_;
      off_count_ = 0;
      if (!state_ && on_count_ >= on_frames_)
      {
        state_ = true;
      }
    }
    else
    {
      ++off_count_;
      on_count_ = 0;
      if (state_ && off_count_ >= off_frames_)
      {
        state_ = false;
      }
    }
    return state_;
  }
  bool state() const { return state_; }

private:
  int on_frames_, off_frames_;
  int on_count_ = 0, off_count_ = 0;
  bool state_ = false;
};

// 2D "obstacle scan": nearest Obstacle point per bearing bin (1 deg). Lets
// Nav2 see low objects that the level scan misses (optional, see CHANGES.md).
class ObstacleScan
{
public:
  explicit ObstacleScan(int bins = 360) : ranges_(bins, kEmpty) {}
  void reset() { std::fill(ranges_.begin(), ranges_.end(), kEmpty); }
  void add(float x, float y)
  {
    const int n = static_cast<int>(ranges_.size());
    // Bin i is centred on bearing -pi + i*2pi/n, matching LaserScan sample i.
    int i = static_cast<int>(std::lround((std::atan2(y, x) + kPi) / (2.0 * kPi) * n)) % n;
    ranges_[i] = std::min(ranges_[i], static_cast<float>(std::hypot(x, y)));
  }
  const std::vector<float> &ranges() const { return ranges_; }
  static constexpr float kEmpty = 1e9f;

private:
  std::vector<float> ranges_;
};

// Height above ground of a beam pointing `beam_deg` below horizontal, at forward distance x.
inline double beamHeightAt(double sensor_height, double beam_deg, double x)
{
  return sensor_height - x * std::tan(beam_deg * kPi / 180.0);
}

} // namespace trail_object_detection

#endif
