#include "trail_object_detection/ground_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>
#include <limits>

GroundFilter::GroundFilter() : Node("ground_filter")
{
  RCLCPP_INFO(this->get_logger(), "[DETECTION] ground_filter: starting");

  min_z_ = declare_parameter("min_z", -0.72);
  max_z_ = declare_parameter("max_z", 1.50);
  min_detection_range_ = declare_parameter("min_detection_range", 0.35);
  max_detection_range_ = declare_parameter("max_detection_range", 4.0);
  stop_range_ = declare_parameter("stop_range", 0.45);
  detection_width_ = declare_parameter("detection_width", 1.0);
  min_detection_points_ = declare_parameter("min_detection_points", 3);
  min_stop_points_ = declare_parameter("min_stop_points", 1);

  RCLCPP_INFO(
      this->get_logger(),
      "Ground filter started: z=[%.2f, %.2f], warning range=[%.2f, %.2f] m, stop range=%.2f m, width=%.2f m",
      min_z_, max_z_, min_detection_range_, max_detection_range_, stop_range_, detection_width_);

  // Subscribe to the raw LiDAR point cloud coming from the sim/bridge
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/husky1/scan/points", 10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
      { callback(msg); });

  // Publish the cleaned-up (ground-removed) point cloud
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/husky1/scan/points_filtered", 10);
  obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/husky1/obstacle_detected", 10);
  stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/husky1/obstacle_stop", 10);

  previous_obstacle_state_ = false;
  has_previous_obstacle_state_ = false;
  previous_stop_state_ = false;
  has_previous_stop_state_ = false;

  RCLCPP_INFO(this->get_logger(), "Subscribed to /husky1/scan/points");
  RCLCPP_INFO(this->get_logger(), "Publishing filtered cloud on /husky1/scan/points_filtered");
  RCLCPP_INFO(this->get_logger(), "Publishing obstacle state on /husky1/obstacle_detected");
  RCLCPP_INFO(this->get_logger(), "Publishing stop state on /husky1/obstacle_stop");
  RCLCPP_INFO(this->get_logger(), "[DETECTION] ground_filter: started");
}

void GroundFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Copy the incoming message so we keep its structure/header intact
  sensor_msgs::msg::PointCloud2 filtered = *msg;
  std::size_t total_points = 0;
  std::size_t retained_points = 0;
  std::size_t detection_points = 0;
  std::size_t stop_points = 0;

  // Iterate over every point's z value (height relative to the sensor)
  sensor_msgs::PointCloud2Iterator<float> iter_z(filtered, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_x(filtered, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(filtered, "y");

  for (; iter_z != iter_z.end(); ++iter_z, ++iter_x, ++iter_y)
  {
    ++total_points;
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

    // Keep only points in the useful obstacle-height band. NaN points remain
    // invalid and are ignored by RViz and downstream point-cloud consumers.
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        z < min_z_ || z > max_z_)
    {
      *iter_z = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    ++retained_points;
    const double range = std::hypot(static_cast<double>(x), static_cast<double>(y));
    if (x > 0.0 && range >= min_detection_range_ &&
        range <= max_detection_range_ && std::abs(y) <= detection_width_ / 2.0)
    {
      ++detection_points;
    }
    if (x > 0.0 && range <= stop_range_ && std::abs(y) <= detection_width_ / 2.0)
    {
      ++stop_points;
    }
  }

  std_msgs::msg::Bool obstacle;
  obstacle.data = detection_points >= static_cast<std::size_t>(min_detection_points_);
  obstacle_pub_->publish(obstacle);

  std_msgs::msg::Bool stop;
  stop.data = stop_points >= static_cast<std::size_t>(min_stop_points_);
  stop_pub_->publish(stop);

  if (!has_previous_obstacle_state_ || obstacle.data != previous_obstacle_state_)
  {
    RCLCPP_INFO(
        this->get_logger(), "Obstacle state changed: %s (corridor points=%zu)",
        obstacle.data ? "DETECTED" : "CLEAR", detection_points);
    previous_obstacle_state_ = obstacle.data;
    has_previous_obstacle_state_ = true;
  }

  if (!has_previous_stop_state_ || stop.data != previous_stop_state_)
  {
    RCLCPP_WARN(
        this->get_logger(), "Stop state changed: %s (near points=%zu)",
        stop.data ? "STOP" : "CLEAR", stop_points);
    previous_stop_state_ = stop.data;
    has_previous_stop_state_ = true;
  }

  RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Cloud: total=%zu retained=%zu warning_points=%zu near_stop_points=%zu warning_threshold=%d stop_threshold=%d warning=%s stop=%s",
      total_points, retained_points, detection_points, stop_points,
      min_detection_points_, min_stop_points_, obstacle.data ? "true" : "false",
      stop.data ? "true" : "false");

  // Publish the filtered cloud for RViz, Nav2, and other consumers.
  pub_->publish(filtered);
}

// Standard ROS 2 node entry point
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}