#ifndef TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_
#define TRAIL_OBJECT_DETECTION__GROUND_FILTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "trail_object_detection/obstacle_logic.hpp"
#include "trail_object_detection/scenery_map.hpp"
#include "trail_object_detection/trail_logic.hpp"

#include <utility>
#include <vector>

// GroundFilter (same node/executable name as before).
//   in : /husky1/scan/points            raw multi-ring cloud
//        /husky1/camera/image           RGB camera: obstacles standing on grass beside the trail are ignored
//        scenery_map (parameter)        Phase 1 SLAM map: obstacles already there when the trails were mapped
//                                       (e.g. tree foliage reaching over the trail) are ignored too
//        /husky1/yolo_image             YOLO-annotated camera, for the snapshot
//   out: /husky1/scan/points_filtered   obstacles + overhead only (ground/self removed)
//        /husky1/obstacle_detected      warn  (Bool, every frame)
//        /husky1/obstacle_stop          stop  (Bool, every frame) -> mission_manager
//        /husky1/obstacle_scan          LaserScan of low+tall obstacles (optional for Nav2)
//        /husky1/obstacle_near          ANY obstacle close ahead, scenery included (Bool, every frame) -> water_guard's
//                                       trail-keeper never steers then: Nav2 has to be free to steer round it
//        /husky1/collision_stop         ANY obstacle right at the bumper (Bool, every frame) -> water_guard blocks
//                                       forward driving, so ignoring scenery for the stop never means hitting it
//        /husky1/obstacle_image         camera snapshot (Image, once per stop activation)
//        /husky1/trail_check_debug      camera image with the checked ground patches painted (only while subscribed)
class GroundFilter : public rclcpp::Node
{
public:
  GroundFilter();

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool trailImageUsable(const rclcpp::Time &scan_stamp);
  trail_object_detection::TrailCheck checkOnTrail(float x, float y);
  bool isOffTrail(float x, float y, bool use_camera, bool use_memory, double now);
  void publishTrailDebug(const std::vector<std::pair<float, float>> &points);

  trail_object_detection::Params params_;
  int min_warn_points_;
  int min_stop_points_;
  double cloud_max_z_;
  trail_object_detection::Debouncer warn_debounce_;
  trail_object_detection::Debouncer stop_debounce_;
  trail_object_detection::Debouncer near_debounce_;
  trail_object_detection::Debouncer collision_debounce_;
  double near_distance_ = 1.5;      // ahead of the bumper, corridor width
  double collision_distance_ = 0.3; // ahead of the bumper...
  double collision_half_width_ = 0.40; // ...across the body (half width 0.34) plus a margin
  bool previous_collision_ = false;
  trail_object_detection::ObstacleScan obstacle_scan_;
  bool previous_warn_ = false;
  bool previous_stop_ = false;

  // Camera trail check: LiDAR finds obstacles, the camera says whether they are on the trail
  trail_object_detection::TrailParams trail_params_;
  bool trail_check_ = true;
  double trail_image_max_age_ = 1.0;  // sim seconds between the camera frame and the LiDAR scan it judges
  double trail_judge_half_width_ = 2.0; // obstacles this far either side are judged in advance (for the memory)
  sensor_msgs::msg::Image::ConstSharedPtr trail_image_;
  trail_object_detection::TrailMemory trail_memory_;
  std::string odom_frame_ = "husky1_odom";
  // LiDAR frame -> odom for the current scan, so verdicts are remembered in a fixed frame
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_cos_ = 1.0, odom_sin_ = 0.0;
  // Known scenery (Phase 1 map), looked up in the map frame
  trail_object_detection::SceneryMap scenery_;
  std::string map_frame_ = "husky1_map";
  double map_x_ = 0.0, map_y_ = 0.0, map_cos_ = 1.0, map_sin_ = 0.0;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  int trail_channels_ = 3;
  bool trail_bgr_ = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr trail_camera_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trail_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obstacle_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr near_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  bool awaiting_obstacle_image_ = false;
};

#endif
