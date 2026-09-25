#include "trail_object_detection/ground_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

GroundFilter::GroundFilter() : Node("ground_filter")
{
  // Hardcoded threshold — sensor mounted 0.68m up, ground reads ~ -0.68,
  // so -0.60 keeps an 8cm buffer above true ground level.
  // Adjust this value directly here to tune filtering strength.
  min_z_ = -0.72;

  RCLCPP_INFO(this->get_logger(), "Ground filter started with min_z = %.2f", min_z_);

  // Subscribe to the raw LiDAR point cloud coming from the sim/bridge
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/husky1/scan/points", 10,
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { callback(msg); });

  // Publish the cleaned-up (ground-removed) point cloud
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/husky1/scan/points_filtered", 10);
}

void GroundFilter::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Copy the incoming message so we keep its structure/header intact
  sensor_msgs::msg::PointCloud2 filtered = *msg;

  // Iterate over every point's z value (height relative to the sensor)
  sensor_msgs::PointCloud2Iterator<float> iter_z(filtered, "z");

  for (; iter_z != iter_z.end(); ++iter_z) {
    // If a point is at or below the ground threshold, mark it invalid.
    // NaN points are automatically ignored by RViz, Nav2, etc.
    if (*iter_z < min_z_) {
      *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  // Publish the filtered cloud for other nodes (obstacle detector, Nav2) to use
  pub_->publish(filtered);
}

// Standard ROS 2 node entry point
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}