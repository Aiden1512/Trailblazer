#ifndef TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_
#define TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

#include "trail_object_detection/obstacle_logic.hpp"

// GroundFilter (same node/executable name as before).
//   in : /husky1/scan/points            raw multi-ring cloud
//   out: /husky1/scan/points_filtered   obstacles + overhead only (ground/self removed)
//        /husky1/obstacle_detected      warn  (Bool, every frame)
//        /husky1/obstacle_stop          stop  (Bool, every frame) -> mission_manager
//        /husky1/obstacle_scan          LaserScan of low+tall obstacles (optional for Nav2)
//        /husky1/obstacle_image         camera snapshot (Image, once per stop activation)
class GroundFilter : public rclcpp::Node
{
public:
  GroundFilter();

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  trail_object_detection::Params params_;
  int min_warn_points_;
  int min_stop_points_;
  double cloud_max_z_;
  trail_object_detection::Debouncer warn_debounce_;
  trail_object_detection::Debouncer stop_debounce_;
  trail_object_detection::ObstacleScan obstacle_scan_;
  bool previous_warn_ = false;
  bool previous_stop_ = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obstacle_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  bool awaiting_obstacle_image_ = false;
};

#endif
