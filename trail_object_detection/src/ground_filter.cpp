#include "trail_object_detection/ground_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <cstddef>
#include <limits>
#include <string>

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
  near_debounce_ = trail_object_detection::Debouncer(1, off_frames);
  collision_debounce_ = trail_object_detection::Debouncer(1, off_frames); // react on the first frame
  near_distance_ = declare_parameter("near_distance", near_distance_);
  collision_distance_ = declare_parameter("collision_distance", collision_distance_);
  collision_half_width_ = declare_parameter("collision_half_width", collision_half_width_);

  auto &t = trail_params_;
  trail_check_ = declare_parameter("trail_check", trail_check_);
  trail_image_max_age_ = declare_parameter("trail_image_max_age", trail_image_max_age_);
  trail_judge_half_width_ = declare_parameter("trail_judge_half_width", trail_judge_half_width_);
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  trail_memory_ = trail_object_detection::TrailMemory(
      0.2, declare_parameter("trail_memory_sec", 120.0));
  map_frame_ = declare_parameter("map_frame", map_frame_);
  const std::string scenery_map = declare_parameter("scenery_map", std::string());
  const double scenery_tolerance = declare_parameter("scenery_tolerance", 0.35);
  if (!scenery_map.empty())
  {
    std::string error;
    if (scenery_.load(scenery_map, scenery_tolerance, error))
    {
      RCLCPP_INFO(get_logger(), "Known scenery: %d occupied cells from '%s' (+%.2f m); obstacles there don't stop the robot",
                  scenery_.occupiedCells(), scenery_map.c_str(), scenery_tolerance);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Could not load scenery map '%s': %s; every obstacle counts",
                   scenery_map.c_str(), error.c_str());
    }
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  t.camera_height = declare_parameter("camera_height", t.camera_height);
  t.horizontal_fov = declare_parameter("horizontal_fov", t.horizontal_fov);
  t.camera_ahead_of_sensor = declare_parameter("camera_ahead_of_sensor", t.camera_ahead_of_sensor);
  t.hue_min = declare_parameter("trail_hue_min", t.hue_min);
  t.hue_max = declare_parameter("trail_hue_max", t.hue_max);
  t.sat_min = declare_parameter("trail_sat_min", t.sat_min);
  t.val_min = declare_parameter("trail_val_min", t.val_min);
  t.min_trail_fraction = declare_parameter("min_trail_fraction", t.min_trail_fraction);

  RCLCPP_INFO(
      get_logger(),
      "Perception: obstacle = %.2f m above ground (+%.2f/m) up to z=%.2f (robot top); "
      "corridor +-%.2f m, stop %.2f m / warn %.2f m ahead of bumper; need %d/%d pts, debounce %d on/%d off",
      p.ground_margin, p.slope_tolerance, p.overhead_z, p.corridor_half_width,
      p.stop_distance, p.warn_distance, min_stop_points_, min_warn_points_, on_frames, off_frames);
  RCLCPP_INFO(
      get_logger(),
      "Trail check %s: obstacles only count if the ground in front of them is trail colour "
      "(hue %.0f-%.0f deg, >= %.0f%% of the patch); LiDAR only when the camera can't see it",
      trail_check_ ? "ON" : "OFF", t.hue_min, t.hue_max, t.min_trail_fraction * 100.0);

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/husky1/scan/points", 10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { callback(msg); });
  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/husky1/yolo_image", rclcpp::SensorDataQoS(),
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
  trail_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/husky1/camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        const std::string &enc = msg->encoding;
        if (enc != "rgb8" && enc != "bgr8" && enc != "rgba8" && enc != "bgra8")
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Trail check: unsupported image encoding '%s'", enc.c_str());
          return;
        }
        if (msg->data.size() < static_cast<std::size_t>(msg->step) * msg->height)
        {
          return;
        }
        trail_channels_ = enc.size() == 5 ? 4 : 3; // "rgba8"/"bgra8" vs "rgb8"/"bgr8"
        trail_bgr_ = enc[0] == 'b';
        trail_image_ = msg;
      });
  trail_debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/husky1/trail_check_debug", 1);
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/husky1/scan/points_filtered", 10);
  obstacle_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/husky1/obstacle_image", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_detected", 10);
  stop_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_stop", 10);
  near_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/obstacle_near", 10);
  collision_pub_ = create_publisher<std_msgs::msg::Bool>("/husky1/collision_stop", 10);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/husky1/obstacle_scan", 10);
  RCLCPP_INFO(
      get_logger(),
      "Obstacle snapshots: taking the next /husky1/yolo_image frame when stop activates; publishing to "
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
  std::size_t off_trail = 0;
  std::size_t scenery = 0;
  std::size_t near_points = 0;      // every obstacle, before the scenery / trail checks
  std::size_t collision_points = 0;
  std::vector<std::pair<float, float>> zone_points; // obstacle points inside the warn/stop corridor
  obstacle_scan_.reset();

  // Camera verdicts are remembered in the odom frame; without TF the memory just isn't used this frame
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double now = stamp.seconds();
  const bool use_camera = trail_check_ && trailImageUsable(stamp);
  bool use_memory = false;
  bool use_scenery = false;
  if (scenery_.loaded())
  {
    try
    {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero);
      const auto &q = tf.transform.rotation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      map_x_ = tf.transform.translation.x;
      map_y_ = tf.transform.translation.y;
      map_cos_ = std::cos(yaw);
      map_sin_ = std::sin(yaw);
      use_scenery = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Known scenery: no TF %s -> %s yet, so every obstacle counts: %s",
                           msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    }
  }
  if (trail_check_)
  {
    try
    {
      const auto tf = tf_buffer_->lookupTransform(odom_frame_, msg->header.frame_id, tf2::TimePointZero);
      const auto &q = tf.transform.rotation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      odom_x_ = tf.transform.translation.x;
      odom_y_ = tf.transform.translation.y;
      odom_cos_ = std::cos(yaw);
      odom_sin_ = std::sin(yaw);
      use_memory = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Trail check: no TF %s -> %s, so no memory of judged obstacles: %s",
                           msg->header.frame_id.c_str(), odom_frame_.c_str(), ex.what());
    }
  }

  for (; ix != ix.end(); ++ix, ++iy, ++iz)
  {
    const float x = *ix, y = *iy, z = *iz;
    const PointClass c = trail_object_detection::classify(x, y, z, params_);

    if (c == PointClass::Obstacle)
    {
      // Physical clearance, whatever the obstacle is: these never get filtered
      const double ahead = static_cast<double>(x) - params_.sensor_to_front;
      if (x > 0.0f && ahead <= near_distance_ && std::fabs(y) <= params_.corridor_half_width)
      {
        ++near_points;
      }
      if (x > 0.0f && ahead <= collision_distance_ && std::fabs(y) <= collision_half_width_)
      {
        ++collision_points;
      }
      ZoneCounts point;
      trail_object_detection::countPoint(x, y, params_, point);
      if (point.warn > 0) // in the corridor ahead: only counts if it is on the trail (or nobody can tell)
      {
        zone_points.emplace_back(x, y);
        if (use_scenery && scenery_.isScenery(map_x_ + map_cos_ * x - map_sin_ * y,
                                              map_y_ + map_sin_ * x + map_cos_ * y))
        {
          ++scenery; // was there when the trails were mapped, e.g. tree foliage over the trail edge
        }
        else if (isOffTrail(x, y, use_camera, use_memory, now))
        {
          ++off_trail;
        }
        else
        {
          counts.warn += point.warn;
          counts.stop += point.stop;
        }
      }
      else if (use_camera && use_memory && x > 0.0f &&
               x <= params_.sensor_to_front + params_.warn_distance &&
               std::fabs(y) <= trail_judge_half_width_)
      {
        // Beside the corridor: judge it now, while the camera can see it, in case a turn swings it into
        // the corridor at close range where the camera can't see the ground in front of it any more.
        isOffTrail(x, y, use_camera, use_memory, now);
      }
      obstacle_scan_.add(x, y); // Nav2 still sees everything, so it steers clear of off-trail obstacles
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
  m.data = near_debounce_.update(near_points >= static_cast<std::size_t>(min_warn_points_));
  near_pub_->publish(m);
  const bool collision = collision_debounce_.update(collision_points >= static_cast<std::size_t>(min_stop_points_));
  m.data = collision;
  collision_pub_->publish(m);
  if (collision != previous_collision_)
  {
    RCLCPP_WARN(get_logger(), "Collision stop %s (%zu pts within %.2f m of the bumper)",
                collision ? "ACTIVE: forward driving blocked" : "CLEAR", collision_points, collision_distance_);
    previous_collision_ = collision;
  }

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
  if ((off_trail > 0 || scenery > 0) && counts.stop < static_cast<std::size_t>(min_stop_points_))
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignoring %zu obstacle points ahead: %zu known scenery from the trail map, "
                         "%zu beside the trail", off_trail + scenery, scenery, off_trail);
  }
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                        "kept=%zu warn_pts=%zu stop_pts=%zu off_trail=%zu scenery=%zu",
                        kept, counts.warn, counts.stop, off_trail, scenery);
  if (use_camera && trail_debug_pub_->get_subscription_count() > 0)
  {
    publishTrailDebug(zone_points);
  }
  if (use_memory)
  {
    trail_memory_.prune(now);
  }

  pub_->publish(filtered);
}

// A camera frame taken close enough to this LiDAR scan to judge it by. Compared by message stamps (sim
// time): a slow sim leaves long wall-clock gaps between camera frames, but the robot hasn't moved in them.
bool GroundFilter::trailImageUsable(const rclcpp::Time &scan_stamp)
{
  if (!trail_image_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Trail check: no frame on /husky1/camera/image yet; using LiDAR only");
    return false;
  }
  const rclcpp::Time image_stamp(trail_image_->header.stamp, scan_stamp.get_clock_type());
  const double age = std::fabs((scan_stamp - image_stamp).seconds());
  if (age > trail_image_max_age_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Trail check: camera frame is %.1f s from the LiDAR scan; using remembered verdicts / LiDAR only",
                         age);
    return false;
  }
  return true;
}

// Is the obstacle point (LiDAR frame) beside the trail? The camera decides when it can see the ground in
// front of the point, and its verdict is remembered; otherwise the remembered verdict for that spot is used.
// With neither, the point counts as on the trail, so the robot still stops for it.
bool GroundFilter::isOffTrail(float x, float y, bool use_camera, bool use_memory, double now)
{
  const double ox = odom_x_ + odom_cos_ * x - odom_sin_ * y;
  const double oy = odom_y_ + odom_sin_ * x + odom_cos_ * y;
  if (use_camera)
  {
    const auto verdict = checkOnTrail(x, y);
    if (verdict != trail_object_detection::TrailCheck::Unknown)
    {
      if (use_memory)
      {
        trail_memory_.mark(ox, oy, verdict == trail_object_detection::TrailCheck::OnTrail, now);
      }
      return verdict == trail_object_detection::TrailCheck::OffTrail;
    }
  }
  return use_memory && trail_memory_.offTrail(ox, oy, now);
}

trail_object_detection::TrailCheck GroundFilter::checkOnTrail(float x, float y)
{
  return trail_object_detection::checkOnTrail(
      trail_image_->data.data(), static_cast<int>(trail_image_->width), static_cast<int>(trail_image_->height),
      static_cast<int>(trail_image_->step), trail_channels_, trail_bgr_, x, y, trail_params_);
}

// Checked ground samples painted green (trail colour) or magenta (not trail).
void GroundFilter::publishTrailDebug(const std::vector<std::pair<float, float>> &points)
{
  sensor_msgs::msg::Image out = *trail_image_;
  const int ri = trail_bgr_ ? 2 : 0, bi = trail_bgr_ ? 0 : 2;
  for (const auto &[x, y] : points)
  {
    trail_object_detection::checkOnTrail(
        trail_image_->data.data(), static_cast<int>(out.width), static_cast<int>(out.height),
        static_cast<int>(out.step), trail_channels_, trail_bgr_, x, y, trail_params_,
        [&](int u, int v, bool is_trail) {
          std::uint8_t *px = out.data.data() + static_cast<std::size_t>(v) * out.step + u * trail_channels_;
          px[ri] = is_trail ? 0 : 255;
          px[1] = is_trail ? 255 : 0;
          px[bi] = is_trail ? 0 : 255;
        });
  }
  trail_debug_pub_->publish(out);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}
