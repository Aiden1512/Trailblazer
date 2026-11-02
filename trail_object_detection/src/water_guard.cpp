#include "trail_object_detection/water_guard.hpp"

#include <chrono>
#include <cmath>
#include <string>

using trail_object_detection::limitForward;

WaterGuard::WaterGuard()
    : Node("water_guard"), last_frame_time_(0, 0, RCL_STEADY_TIME),
      follow_trail_time_(0, 0, RCL_STEADY_TIME), obstacle_time_(0, 0, RCL_STEADY_TIME)
{
  auto &p = params_;
  p.camera_height = declare_parameter("camera_height", p.camera_height);
  p.horizontal_fov = declare_parameter("horizontal_fov", p.horizontal_fov);
  p.hue_min = declare_parameter("hue_min", p.hue_min);
  p.hue_max = declare_parameter("hue_max", p.hue_max);
  p.sat_min = declare_parameter("sat_min", p.sat_min);
  p.val_min = declare_parameter("val_min", p.val_min);
  p.corridor_half_width = declare_parameter("corridor_half_width", p.corridor_half_width);
  p.stop_distance = declare_parameter("stop_distance", p.stop_distance);
  p.release_distance = declare_parameter("release_distance", p.release_distance);
  p.slow_distance = declare_parameter("slow_distance", p.slow_distance);
  p.min_speed_factor = declare_parameter("min_speed_factor", p.min_speed_factor);
  p.max_range = declare_parameter("max_range", p.max_range);
  p.min_pixels_per_row = declare_parameter("min_pixels_per_row", p.min_pixels_per_row);
  p.pixel_stride = declare_parameter("pixel_stride", p.pixel_stride);
  // Wall-clock seconds. The camera is 3 Hz in sim time, but a slow sim
  // (software rendering) can leave >1.5 s between frames in real time.
  camera_timeout_ = declare_parameter("camera_timeout", 3.0);
  block_without_camera_ = declare_parameter("block_without_camera", true);
  gate_ = trail_object_detection::WaterGate(p);

  auto &k = keeper_params_;
  trail_keeper_ = declare_parameter("trail_keeper", trail_keeper_);
  follow_trail_timeout_ = declare_parameter("follow_trail_timeout", follow_trail_timeout_);
  obstacle_hold_sec_ = declare_parameter("keeper_obstacle_hold_sec", obstacle_hold_sec_);
  k.camera.camera_height = p.camera_height;
  k.camera.horizontal_fov = p.horizontal_fov;
  k.camera.hue_min = declare_parameter("trail_hue_min", k.camera.hue_min);
  k.camera.hue_max = declare_parameter("trail_hue_max", k.camera.hue_max);
  k.camera.sat_min = declare_parameter("trail_sat_min", k.camera.sat_min);
  k.camera.val_min = declare_parameter("trail_val_min", k.camera.val_min);
  k.on_trail_fraction = declare_parameter("keeper_on_trail_fraction", k.on_trail_fraction);
  k.max_angular = declare_parameter("keeper_max_angular", k.max_angular);
  k.off_trail_speed_factor = declare_parameter("keeper_speed_factor", k.off_trail_speed_factor);
  if (block_without_camera_)
  {
    gate_.blind(); // nothing seen yet
  }

  RCLCPP_INFO(
      get_logger(),
      "Water guard: hue %.0f-%.0f deg, sat >= %.2f, val >= %.2f; corridor +-%.2f m, "
      "stop %.2f m (release %.2f m) / slow %.2f m ahead of the camera; %s without camera",
      p.hue_min, p.hue_max, p.sat_min, p.val_min, p.corridor_half_width, p.stop_distance,
      p.release_distance, p.slow_distance, block_without_camera_ ? "forward blocked" : "no limit");

  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/husky1/camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/husky1/cmd_vel_teleop", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { onCmd(msg); });
  nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/husky1/cmd_vel_nav_smoothed", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { onNavCmd(msg); });
  follow_trail_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/husky1/follow_trail", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
        follow_trail_ = msg->data;
        follow_trail_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
      });
  obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/husky1/obstacle_detected", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data)
        {
          obstacle_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
          have_obstacle_time_ = true;
        }
      });
  keeper_active_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/trail_keeper_active", 10);
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/husky1/cmd_vel", 10);
  warn_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/water_detected", 10);
  stop_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/water_stop", 10);
  distance_pub_ = create_publisher<std_msgs::msg::Float32>("/husky1/water_distance", 10);
  debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/husky1/water_debug", 1);
  watchdog_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { checkCamera(); });

  RCLCPP_INFO(get_logger(), "Forwarding /husky1/cmd_vel_teleop and /husky1/cmd_vel_nav_smoothed (Nav2) -> /husky1/cmd_vel");
  RCLCPP_INFO(get_logger(), "Trail-keeper %s: steers Nav2 back onto the trail when less than %.0f%% of the path "
                            "ahead is trail colour (while /husky1/follow_trail is true and no obstacle is near)",
              trail_keeper_ ? "ON" : "OFF", k.on_trail_fraction * 100.0);
}

void WaterGuard::onCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_ = *msg;
  have_cmd_ = true;
  if (msg->linear.x > 0.0 && gate_.stopped())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Forward blocked (%s); reverse or turn away",
                         blind_ ? "no camera" : "water ahead");
  }
  publishGated();
}

// Nav2 commands: let the trail-keeper replace the steering if Nav2 is cutting across the grass, then
// apply the water limit like any other command.
void WaterGuard::onNavCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  trail_object_detection::Steering steer;
  if (trailKeeperAllowed())
  {
    steer = trail_object_detection::keepOnTrail(trail_view_, msg->linear.x, msg->angular.z, keeper_params_);
  }
  if (steer.active)
  {
    msg->linear.x *= steer.speed_factor;
    msg->angular.z = steer.angular;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Trail-keeper: only %.0f%% of the path ahead is trail; steering back onto it "
                         "(%.2f m %s, %.2f rad/s)",
                         trail_view_.ahead_fraction * 100.0, std::fabs(steer.target),
                         steer.target >= 0.0 ? "left" : "right", steer.angular);
  }
  if (steer.active != keeper_active_)
  {
    keeper_active_ = steer.active;
    std_msgs::msg::Bool b;
    b.data = keeper_active_;
    keeper_active_pub_->publish(b);
    if (!keeper_active_)
    {
      RCLCPP_INFO(get_logger(), "Trail-keeper: back on the trail, Nav2 steering again");
    }
  }
  onCmd(msg);
}

// Only while mission_manager says we are following a trail, with a current camera view, and not near an
// obstacle (Nav2 has to leave the trail to get round one, and the keeper would steer it back into it).
bool WaterGuard::trailKeeperAllowed()
{
  if (!trail_keeper_ || !have_frame_ || blind_ || !trail_view_.valid)
  {
    return false;
  }
  const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
  if (!follow_trail_ || (now - follow_trail_time_).seconds() > follow_trail_timeout_)
  {
    return false;
  }
  return !have_obstacle_time_ || (now - obstacle_time_).seconds() > obstacle_hold_sec_;
}

void WaterGuard::publishGated()
{
  geometry_msgs::msg::Twist out = last_cmd_;
  out.linear.x = limitForward(out.linear.x, gate_.factor());
  cmd_pub_->publish(out);
  last_published_x_ = out.linear.x;
}

// The diff-drive keeps executing the last command it got, so a single tap of
// "forward" would carry the robot into the water. When the limit tightens,
// re-send the operator's last command with the new limit applied.
void WaterGuard::enforceLimit(const char *reason)
{
  if (have_cmd_ && limitForward(last_cmd_.linear.x, gate_.factor()) < last_published_x_ - 1e-6)
  {
    publishGated();
    RCLCPP_INFO(get_logger(), "Forward speed limited to %.2f m/s (%s)", last_published_x_, reason);
  }
}

void WaterGuard::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  int channels;
  bool bgr;
  const std::string &enc = msg->encoding;
  if (enc == "rgb8" || enc == "bgr8")
  {
    channels = 3;
  }
  else if (enc == "rgba8" || enc == "bgra8")
  {
    channels = 4;
  }
  else
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported image encoding '%s'",
                          enc.c_str());
    return;
  }
  bgr = enc[0] == 'b';
  if (msg->data.size() < static_cast<std::size_t>(msg->step) * msg->height)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image data shorter than step * height");
    return;
  }

  const auto scan = trail_object_detection::scanForWater(
      msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
      static_cast<int>(msg->step), channels, bgr, params_);
  gate_.update(scan.nearest);
  if (trail_keeper_)
  {
    trail_view_ = trail_object_detection::viewTrail(
        msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height),
        static_cast<int>(msg->step), channels, bgr, keeper_params_);
  }
  last_frame_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  have_frame_ = true;
  if (blind_)
  {
    blind_ = false;
    RCLCPP_INFO(get_logger(), "Camera frames arriving");
    // The stop was only for the camera gap, not something the operator saw or
    // asked for, so carry on with their last command if the way is clear.
    if (have_cmd_ && limitForward(last_cmd_.linear.x, gate_.factor()) > last_published_x_ + 1e-6)
    {
      publishGated();
      RCLCPP_INFO(get_logger(), "Resumed at %.2f m/s after camera gap", last_published_x_);
    }
  }
  enforceLimit("water ahead");

  const bool stop = gate_.stopped();
  const bool warn = stop || gate_.slowed();
  std_msgs::msg::Bool b;
  b.data = warn;
  warn_pub_->publish(b);
  b.data = stop;
  stop_pub_->publish(b);
  std_msgs::msg::Float32 dist;
  dist.data = std::isfinite(scan.nearest) ? static_cast<float>(scan.nearest) : -1.0f;
  distance_pub_->publish(dist);

  if (warn != previous_warn_)
  {
    RCLCPP_INFO(get_logger(), "Water ahead %s (%.2f m)", warn ? "DETECTED" : "CLEAR", scan.nearest);
    previous_warn_ = warn;
  }
  if (stop != previous_stop_)
  {
    RCLCPP_WARN(get_logger(), "Water stop %s (%.2f m)", stop ? "ACTIVE" : "CLEAR", scan.nearest);
    previous_stop_ = stop;
  }
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "nearest=%.2f corridor_px=%d factor=%.2f",
                        scan.nearest, scan.corridor_pixels, gate_.factor());

  if (debug_pub_->get_subscription_count() > 0)
  {
    publishDebug(*msg, channels, bgr);
  }
}

void WaterGuard::checkCamera()
{
  if (!block_without_camera_)
  {
    return;
  }
  const double age = have_frame_
      ? (rclcpp::Clock(RCL_STEADY_TIME).now() - last_frame_time_).seconds()
      : camera_timeout_ + 1.0;
  if (age <= camera_timeout_)
  {
    return;
  }
  if (!blind_)
  {
    blind_ = true;
    RCLCPP_WARN(get_logger(), "No camera frames on /husky1/camera/image; forward driving blocked");
  }
  gate_.blind();
  enforceLimit("no camera");
}

// Water samples in the corridor painted red, outside it yellow; the stop and
// slow distances drawn as white / orange lines across the corridor.
void WaterGuard::publishDebug(const sensor_msgs::msg::Image &msg, int channels, bool bgr)
{
  sensor_msgs::msg::Image out = msg;
  const int w = static_cast<int>(out.width), h = static_cast<int>(out.height);
  const int ri = bgr ? 2 : 0, bi = bgr ? 0 : 2;
  const int stride = std::max(1, params_.pixel_stride);
  auto paint = [&](int u, int v, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    if (u < 0 || v < 0 || u >= w || v >= h)
    {
      return;
    }
    std::uint8_t *px = out.data.data() + static_cast<std::size_t>(v) * out.step + u * channels;
    px[ri] = r;
    px[1] = g;
    px[bi] = b;
  };

  trail_object_detection::scanForWater(
      msg.data.data(), w, h, static_cast<int>(msg.step), channels, bgr, params_,
      [&](int u, int v, bool in_corridor) {
        for (int dv = 0; dv < stride; ++dv)
        {
          for (int du = 0; du < stride; ++du)
          {
            if (in_corridor)
            {
              paint(u + du, v + dv, 255, 0, 0);
            }
            else
            {
              paint(u + du, v + dv, 255, 220, 0);
            }
          }
        }
      });

  const double f = (w / 2.0) / std::tan(params_.horizontal_fov / 2.0);
  auto line = [&](double d, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    const int v = trail_object_detection::rowForDistance(d, w, h, params_);
    if (v < 0)
    {
      return;
    }
    const double half_px = params_.corridor_half_width * f / d;
    for (int u = static_cast<int>(w / 2.0 - half_px); u <= static_cast<int>(w / 2.0 + half_px); ++u)
    {
      paint(u, v, r, g, b);
      paint(u, v + 1, r, g, b);
    }
  };
  line(params_.stop_distance, 255, 255, 255);
  line(params_.slow_distance, 255, 140, 0);

  debug_pub_->publish(out);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaterGuard>());
  rclcpp::shutdown();
  return 0;
}
