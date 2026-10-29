#ifndef TRAIL_OBJECT_DETECTION__WATER_GUARD_HPP_
#define TRAIL_OBJECT_DETECTION__WATER_GUARD_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "trail_object_detection/water_logic.hpp"

// WaterGuard: stops teleop driving into water (blue ground) seen by the camera.
// Teleop publishes to /husky1/cmd_vel_teleop instead of /husky1/cmd_vel; this
// node forwards it, limiting FORWARD speed only, so the operator can always
// reverse or turn away. Nav2 still publishes straight to /husky1/cmd_vel, so
// the Phase 2 autonomous run is unaffected.
//   in : /husky1/camera/image        RGB camera (~3 Hz)
//        /husky1/cmd_vel_teleop      operator commands
//   out: /husky1/cmd_vel             commands with forward speed limited
//        /husky1/water_detected      warn: water inside slow_distance (Bool, every frame)
//        /husky1/water_stop          stop: forward driving blocked    (Bool, every frame)
//        /husky1/water_distance      nearest water ahead [m], -1 = none (Float32, every frame)
//        /husky1/water_debug         camera image with water painted in (only while subscribed)
class WaterGuard : public rclcpp::Node
{
public:
  WaterGuard();

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void checkCamera();
  void publishGated();
  void enforceLimit(const char *reason);
  void publishDebug(const sensor_msgs::msg::Image &msg, int channels, bool bgr);

  trail_object_detection::WaterParams params_;
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr warn_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

#endif
