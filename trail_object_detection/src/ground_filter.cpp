#include "trail_object_detection/ground_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <cstddef>
#include <limits>

using trail_object_detection::PointClass;
using trail_object_detection::ZoneCounts;

GroundFilter::GroundFilter() : Node("ground_filter")
{
  // Parameters, so values can be tuned with ros2 param set without rebuilding
  auto &p = params_;
  p.sensor_height = declare_parameter("sensor_height", p.sensor_height);
  p.ground_margin = declare_parameter("ground_margin", p.ground_margin);
  p.slope_tolerance = declare_parameter("slope_tolerance", p.slope_tolerance);
  p.body_half_length = declare_parameter("body_half_length", p.body_half_length);
  p.body_half_width = declare_parameter("body_half_width", p.body_half_width);
  p.overhead_z = declare_parameter("overhead_z", p.overhead_z);
  p.sensor_to_front = declare_parameter("sensor_to_front", p.sensor_to_front);
  p.corridor_half_width = declare_parameter("corridor_half_width", p.corridor_half_width);
  p.stop_distance = declare_parameter("stop_distance", p.stop_distance);
  p.warn_distance = declare_parameter("warn_distance", p.warn_distance);
  p.max_range = declare_parameter("max_range", p.max_range);
  cloud_max_z_ = declare_parameter("cloud_max_z", 1.5); // Overhead points above this are hidden in RViz

  // At 3 m, 1 deg of beam is about 5 cm, so 3 points is a 15 cm object. Fewer is probably noise
  min_points_ = declare_parameter("min_points", 3);
  // 2 frames to stop (about 0.7 s at 3 Hz), 3 to release (about 1 s) so ring gaps don't flicker the stop
  const int on_frames = declare_parameter("on_frames", 2);
  const int off_frames = declare_parameter("off_frames", 3);
  warn_debounce_ = trail_object_detection::Debouncer(on_frames, off_frames);
  stop_debounce_ = trail_object_detection::Debouncer(on_frames, off_frames);

  RCLCPP_INFO(
      get_logger(),
      "Perception: obstacle = %.2f m above ground (+%.2f/m) up to z=%.2f; corridor +-%.2f m, "
      "stop %.2f m / warn %.2f m from bumper; need %d pts, debounce %d on / %d off",
      p.ground_margin, p.slope_tolerance, p.overhead_z, p.corridor_half_width,
      p.stop_distance, p.warn_distance, min_points_, on_frames, off_frames);

  // Topic names must match what mission_manager and the GUI subscribe to
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/husky1/scan/points", 10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { callback(msg); });
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/husky1/scan/points_filtered", 10);
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_detected", 10);
  stop_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_stop", 10);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/husky1/obstacle_scan", 10);

  // Snapshot for the GUI: after a stop starts, the next camera frame is republished once.
  // Transient local, so a GUI that starts late still gets the last one
  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/husky1/camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!awaiting_obstacle_image_)
        {
          return;
        }
        obstacle_image_pub_->publish(*msg);
        awaiting_obstacle_image_ = false;
        RCLCPP_INFO(
            get_logger(), "Published obstacle snapshot (%ux%u) on /husky1/obstacle_image",
            msg->width, msg->height);
      });
  obstacle_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/husky1/obstacle_image", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
}

// Runs once per cloud (about 3 Hz)
void GroundFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 filtered = *msg;
  filtered.is_dense = false; // Removed points become NaN

  sensor_msgs::PointCloud2Iterator<float> ix(filtered, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(filtered, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(filtered, "z");

  const float nan = std::numeric_limits<float>::quiet_NaN();
  ZoneCounts counts;
  obstacle_scan_.reset();

  // Classify every point. Obstacles are counted and kept, overhead points are only kept for RViz
  for (; ix != ix.end(); ++ix, ++iy, ++iz)
  {
    const float x = *ix, y = *iy, z = *iz;
    const PointClass c = trail_object_detection::classify(x, y, z, params_);

    if (c == PointClass::Obstacle)
    {
      trail_object_detection::countPoint(x, y, params_, counts);
      obstacle_scan_.add(x, y);
    }
    else if (!(c == PointClass::Overhead && z <= cloud_max_z_))
    {
      // NaN all three so anything that only checks x and y skips it too
      *ix = nan;
      *iy = nan;
      *iz = nan;
    }
  }

  const bool warn = warn_debounce_.update(counts.warn >= static_cast<std::size_t>(min_points_));
  const bool stop = stop_debounce_.update(counts.stop >= static_cast<std::size_t>(min_points_));

  // Published every frame, not only on change, so a late subscriber still gets the current state
  std_msgs::msg::Bool m;
  m.data = warn;
  obstacle_pub_->publish(m);
  m.data = stop;
  stop_pub_->publish(m);

  if (stop && !previous_stop_)
  {
    awaiting_obstacle_image_ = true;
    RCLCPP_INFO(get_logger(), "Obstacle stop active; capturing the next camera frame");
  }
  else if (!stop && previous_stop_ && awaiting_obstacle_image_)
  {
    awaiting_obstacle_image_ = false;
    RCLCPP_WARN(get_logger(), "Obstacle stop cleared before a camera frame arrived; snapshot cancelled");
  }

  // 2D obstacle scan, one range per degree. Empty bearings are inf
  sensor_msgs::msg::LaserScan scan;
  scan.header = msg->header;
  scan.angle_min = -static_cast<float>(M_PI);
  scan.angle_max = static_cast<float>(M_PI);
  scan.angle_increment = static_cast<float>(2.0 * M_PI / trail_object_detection::kScanBins);
  scan.range_min = 0.1f;
  scan.range_max = static_cast<float>(params_.max_range);
  for (float r : obstacle_scan_.ranges())
  {
    scan.ranges.push_back(r > scan.range_max ? std::numeric_limits<float>::infinity() : r);
  }
  scan_pub_->publish(scan);

  // Log only on changes to avoid spam
  if (warn != previous_warn_)
  {
    RCLCPP_INFO(get_logger(), "Obstacle warning %s (pts=%zu)", warn ? "DETECTED" : "CLEAR", counts.warn);
    previous_warn_ = warn;
  }
  if (stop != previous_stop_)
  {
    RCLCPP_WARN(get_logger(), "Obstacle stop %s (pts=%zu)", stop ? "ACTIVE" : "CLEAR", counts.stop);
    previous_stop_ = stop;
  }

  pub_->publish(filtered);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}
