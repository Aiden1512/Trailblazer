#ifndef TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_
#define TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

// GroundFilter
// Subscribes to the raw 3D LiDAR point cloud and removes points that
// belong to flat ground, so only real obstacles remain in the output cloud.
// Pre-processing step before obstacle detection / Nav2 costmap consumption.
class GroundFilter : public rclcpp::Node
{
public:
  // Constructor: sets up the threshold, subscriber, and publisher
  GroundFilter();

private:
  // Callback triggered every time a new raw point cloud arrives
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Minimum height (in the sensor's own frame) a point must have to be
  // kept. Points below this are treated as ground and discarded.
  // Sensor is mounted 0.68m above ground, so ground points read ~ -0.68.
  double min_z_;
  double max_z_;
  double min_detection_range_;
  double max_detection_range_;
  double stop_range_;
  double detection_width_;
  int min_detection_points_;
  int min_stop_points_;
  bool previous_obstacle_state_;
  bool has_previous_obstacle_state_;
  bool previous_stop_state_;
  bool has_previous_stop_state_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
};

#endif // TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_