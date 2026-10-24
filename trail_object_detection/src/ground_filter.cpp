#include "trail_object_detection/ground_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <cstddef>
#include <limits>

using trail_object_detection::PointClass;
using trail_object_detection::ZoneCounts;

GroundFilter::GroundFilter() : Node("ground_filter")
{
  auto &p = params_;
  p.sensor_height = declare_parameter("sensor_height", p.sensor_height);
  p.ground_margin = declare_parameter("ground_margin", p.ground_margin);
  p.slope_tolerance = declare_parameter("slope_tolerance", p.slope_tolerance);
  p.body_front = declare_parameter("body_front", p.body_front);
  p.body_rear = declare_parameter("body_rear", p.body_rear);
  p.body_half_width = declare_parameter("body_half_width", p.body_half_width);
  p.overhead_z = declare_parameter("overhead_z", p.overhead_z);
  p.sensor_to_front = declare_parameter("sensor_to_front", p.sensor_to_front);
  p.corridor_half_width = declare_parameter("corridor_half_width", p.corridor_half_width);
  p.stop_distance = declare_parameter("stop_distance", p.stop_distance);
  p.warn_distance = declare_parameter("warn_distance", p.warn_distance);
  p.max_range = declare_parameter("max_range", p.max_range);
  cloud_max_z_ = declare_parameter("cloud_max_z", 1.5);

  min_warn_points_ = declare_parameter("min_warn_points", 3);
  min_stop_points_ = declare_parameter("min_stop_points", 3);
  const int on_frames = declare_parameter("on_frames", 2);
  const int off_frames = declare_parameter("off_frames", 3);
  warn_debounce_ = trail_object_detection::Debouncer(on_frames, off_frames);
  stop_debounce_ = trail_object_detection::Debouncer(on_frames, off_frames);

  RCLCPP_INFO(
      get_logger(),
      "Perception: obstacle = %.2f m above ground (+%.2f/m) up to z=%.2f (robot top); "
      "corridor +-%.2f m, stop %.2f m / warn %.2f m ahead of bumper; need %d/%d pts, debounce %d on/%d off",
      p.ground_margin, p.slope_tolerance, p.overhead_z, p.corridor_half_width,
      p.stop_distance, p.warn_distance, min_stop_points_, min_warn_points_, on_frames, off_frames);

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/husky1/scan/points", 10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { callback(msg); });
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
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/husky1/scan/points_filtered", 10);
  obstacle_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/husky1/obstacle_image", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_detected", 10);
  stop_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_stop", 10);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/husky1/obstacle_scan", 10);
  RCLCPP_INFO(
      get_logger(),
      "Obstacle snapshots: waiting for /husky1/camera/image when stop activates; publishing to "
      "/husky1/obstacle_image");
}

void GroundFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 filtered = *msg;
  filtered.is_dense = false;

  sensor_msgs::PointCloud2Iterator<float> ix(filtered, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(filtered, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(filtered, "z");

  const float nan = std::numeric_limits<float>::quiet_NaN();
  ZoneCounts counts;
  std::size_t kept = 0;
  obstacle_scan_.reset();

  for (; ix != ix.end(); ++ix, ++iy, ++iz)
  {
    const float x = *ix, y = *iy, z = *iz;
    const PointClass c = trail_object_detection::classify(x, y, z, params_);

    if (c == PointClass::Obstacle)
    {
      trail_object_detection::countPoint(x, y, params_, counts);
      obstacle_scan_.add(x, y);
      ++kept;
    }
    else if (c == PointClass::Overhead && z <= cloud_max_z_)
    {
      ++kept; // visible in RViz, never counted
    }
    else
    {
      *ix = nan;
      *iy = nan;
      *iz = nan;
    }
  }

  const bool warn = warn_debounce_.update(counts.warn >= static_cast<std::size_t>(min_warn_points_));
  const bool stop = stop_debounce_.update(counts.stop >= static_cast<std::size_t>(min_stop_points_));

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
    RCLCPP_WARN(
        get_logger(),
        "Obstacle stop cleared before a camera frame arrived; obstacle snapshot cancelled");
  }

  sensor_msgs::msg::LaserScan scan;
  scan.header = msg->header;
  scan.angle_min = -static_cast<float>(trail_object_detection::kPi);
  scan.angle_max = static_cast<float>(trail_object_detection::kPi);
  scan.angle_increment = static_cast<float>(2.0 * trail_object_detection::kPi / 360.0);
  scan.range_min = 0.1f;
  scan.range_max = static_cast<float>(params_.max_range);
  for (float r : obstacle_scan_.ranges())
  {
    scan.ranges.push_back(r > scan.range_max ? std::numeric_limits<float>::infinity() : r);
  }
  scan_pub_->publish(scan);

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
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                        "kept=%zu warn_pts=%zu stop_pts=%zu", kept, counts.warn, counts.stop);

  pub_->publish(filtered);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}
