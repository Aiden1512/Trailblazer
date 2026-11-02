#ifndef TRAIL_OBJECT_DETECTION__WATER_GUARD_HPP_
#define TRAIL_OBJECT_DETECTION__WATER_GUARD_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "trail_object_detection/trail_keeper_logic.hpp"
#include "trail_object_detection/water_logic.hpp"

// WaterGuard: stops the robot driving into water (blue ground) seen by the camera.
// Teleop publishes to /husky1/cmd_vel_teleop and Nav2 to /husky1/cmd_vel_nav_smoothed
// (see 41068_navigation.launch.py) instead of /husky1/cmd_vel; this node forwards
// both, limiting FORWARD speed only, so the operator (or Nav2) can always reverse
// or turn away. If Nav2 keeps pushing towards water it stalls, its goal aborts and
// mission_manager treats that trail as blocked.
// Trail-keeper: Nav2 only knows the waypoints, not the trail, so it cuts corners
// across the grass. While mission_manager says the robot is following a trail
// (/husky1/follow_trail) and no obstacle is near, Nav2's steering is replaced by
// a camera one whenever the path ahead turns from brown to grass, until it is
// back on the trail (see trail_keeper_logic.hpp).
//   in : /husky1/camera/image        RGB camera (~3 Hz)
//        /husky1/cmd_vel_teleop      operator commands
//        /husky1/cmd_vel_nav_smoothed  Nav2 commands (Phase 2 autonomous run)
//        /husky1/follow_trail        mission_manager: trail-keeper may steer (Bool, 5 Hz)
//        /husky1/obstacle_detected   ground_filter warn: trail-keeper stands down near obstacles
//        /husky1/obstacle_near       ground_filter: ANY obstacle close ahead -> trail-keeper stands down
//        /husky1/collision_stop      ground_filter: obstacle at the bumper -> forward driving blocked
//   out: /husky1/cmd_vel             commands with forward speed limited
//        /husky1/water_detected      warn: water inside slow_distance (Bool, every frame)
//        /husky1/water_stop          stop: forward driving blocked    (Bool, every frame)
//        /husky1/water_distance      nearest water ahead [m], -1 = none (Float32, every frame)
//        /husky1/water_debug         camera image with water painted in (only while subscribed)
//        /husky1/water_points        water seen on the ground, one point per water_cell in base_link at
//                                    water_cloud_height (PointCloud2, every frame): Nav2's water_layer marks
//                                    it so the planner routes round water, which the lidar can't see
//        /husky1/trail_keeper_active trail-keeper is steering Nav2 back onto the trail (Bool, on change)
class WaterGuard : public rclcpp::Node
{
public:
  WaterGuard();

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onNavCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool trailKeeperAllowed();
  void checkCamera();
  void publishGated();
  double forwardFactor() const;
  void enforceLimit(const char *reason);
  void publishDebug(const sensor_msgs::msg::Image &msg, int channels, bool bgr);
  void publishWaterPoints(const std_msgs::msg::Header &image_header,
                          const std::vector<std::pair<double, double>> &points);

  trail_object_detection::WaterParams params_;
  std::string base_frame_ = "husky1_base_link";
  double water_cell_ = 0.1;         // one cloud point per water cell this size (m)
  double water_cloud_height_ = 0.3; // above base_link, inside the costmap's obstacle height band
  trail_object_detection::WaterGate gate_;
  double camera_timeout_;
  bool block_without_camera_;

  geometry_msgs::msg::Twist last_cmd_;
  bool have_cmd_ = false;
  double last_published_x_ = 0.0;
  rclcpp::Time last_frame_time_;
  bool have_frame_ = false;
  bool blind_ = false;
  bool previous_stop_ = false;
  bool previous_warn_ = false;

  // Trail-keeper (all times are wall clock, like last_frame_time_)
  trail_object_detection::TrailKeeperParams keeper_params_;
  bool trail_keeper_ = true;
  double follow_trail_timeout_ = 1.0;
  double obstacle_hold_sec_ = 3.0; // stay out of Nav2's way this long after an obstacle warning clears
  trail_object_detection::TrailView trail_view_;
  bool follow_trail_ = false;
  rclcpp::Time follow_trail_time_;
  rclcpp::Time obstacle_time_;
  bool have_obstacle_time_ = false;
  bool keeper_active_ = false;
  bool collision_stop_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follow_trail_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_near_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr keeper_active_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr warn_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr water_points_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

#endif
