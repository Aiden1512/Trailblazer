#ifndef TRAIL_OBJECT_DETECTION__OBSTACLE_LOGIC_HPP_
#define TRAIL_OBJECT_DETECTION__OBSTACLE_LOGIC_HPP_

// Perception core. No ROS in here, so it can be tested without the sim.
// Points are in the sensor frame: x forward, y left, z up, origin at the LiDAR.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace trail_object_detection
{

// One bearing bin per degree, same as the LiDAR's 360 horizontal samples.
constexpr int kScanBins = 360;

// Every tunable number lives here. Defaults come from the Husky and trail_world.
struct Params
{
  // Ground
  double sensor_height = 0.80;   // Measured in the sim, flat ground reads z = -0.80
  double ground_margin = 0.08;   // Above the 1 cm noise, below the 12-20 cm plants. 3 cm pebbles are ignored
  double slope_tolerance = 0.04; // Extra margin per metre of range (about 2 deg of tilt) so far ground isn't an obstacle

  // Robot body, so the LiDAR doesn't detect the Husky itself
  double body_half_length = 0.52; // Collision box is 0.99 m long, plus a small margin
  double body_half_width = 0.34;  // Collision box is 0.57 m wide, plus a small margin

  // Overhead
  double overhead_z = 0.06; // Mast top is about +0.02, plus 4 cm clearance. Higher points pass over the robot. Tune per branch

  // Corridor (distances are measured from the front bumper, so they mean clearance)
  double sensor_to_front = 0.50;     // Sensor to bumper
  double corridor_half_width = 0.45; // Half the robot width (0.285) plus margin
  double stop_distance = 1.0;        // About 2 s at Nav2's 0.5 m/s, covers the debounce delay
  double warn_distance = 3.5;        // Early heads up for the GUI only

  double max_range = 8.0; // Ignore far returns, they only add noise
};

enum class PointClass
{
  Invalid,  // NaN or too far
  Self,     // The robot's own body
  Ground,   // Flat ground or too low to matter
  Overhead, // Above the robot (branches, canopy)
  Obstacle  // Something the robot could hit
};

// Ground margin at a given range. Grows with range so a small tilt doesn't turn far ground into obstacles.
inline double groundMargin(double range, const Params &p)
{
  return std::max(p.ground_margin, p.slope_tolerance * range);
}

// Sorts one point into a class. Order matters: body first, then overhead, then ground.
inline PointClass classify(float fx, float fy, float fz, const Params &p)
{
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz))
  {
    return PointClass::Invalid;
  }
  const double x = fx, y = fy, z = fz;
  const double range = std::hypot(x, y);
  if (range > p.max_range)
  {
    return PointClass::Invalid;
  }
  if (std::fabs(x) < p.body_half_length && std::fabs(y) < p.body_half_width)
  {
    return PointClass::Self;
  }
  if (z > p.overhead_z)
  {
    return PointClass::Overhead;
  }
  if (z < -p.sensor_height + groundMargin(range, p))
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

// Adds an obstacle point to the warn/stop counts if it is in the lane ahead of the bumper.
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

// Needs on_frames positives in a row to switch on and off_frames negatives in a row to switch off.
// mission_manager cancels and resends its Nav2 goal on every change, so flicker would make the robot stutter.
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

private:
  int on_frames_, off_frames_;
  int on_count_ = 0, off_count_ = 0;
  bool state_ = false;
};

// Nearest obstacle per bearing, as a 2D scan. Lets Nav2 or SLAM see low objects the level scan misses.
class ObstacleScan
{
public:
  static constexpr float kEmpty = 1e9f;

  ObstacleScan() : ranges_(kScanBins, kEmpty) {}
  void reset() { std::fill(ranges_.begin(), ranges_.end(), kEmpty); }

  void add(float x, float y)
  {
    // Bin i is centred on bearing -pi + i * 2pi / kScanBins, same as LaserScan sample i
    const int i = static_cast<int>(std::lround((std::atan2(y, x) + M_PI) / (2.0 * M_PI) * kScanBins)) % kScanBins;
    ranges_[i] = std::min(ranges_[i], static_cast<float>(std::hypot(x, y)));
  }

  const std::vector<float> &ranges() const { return ranges_; }

private:
  std::vector<float> ranges_;
};

} // namespace trail_object_detection

#endif
