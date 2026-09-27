#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

struct Waypoint
{
    double x;
    double y;
    double yaw;
};

class PathRecorder : public rclcpp::Node
{
public:
    PathRecorder()
        : Node("path_recorder")
    {
        this->declare_parameter<double>("spacing", 0.5);
        this->declare_parameter<std::string>(
            "save_file",
            "recorded_path.yaml"
        );

        spacing_ = this->get_parameter("spacing").as_double();
        save_file_ = this->get_parameter("save_file").as_string();

        // Subscribing to the SLAM pose
        pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/husky1/pose", 10, std::bind(&PathRecorder::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Path recorder started.");

    }

    ~PathRecorder()
    {
        savePath();
    }

private:


    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    std::vector<Waypoint> waypoints_;

    double spacing_;
    double last_x_ = 0.0;
    double last_y_ = 0.0;

    std::string save_file_;

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Get position of robot 
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Convert quaternion to yaw
        double yaw = getYaw(msg->pose.pose.orientation);

        // Always record the first waypoint
        if (waypoints_.empty())
        {
            addWaypoint(x, y, yaw, msg);
            return;
        }

        // Calculate distance from previous waypoint
        double dx = x - last_x_;
        double dy = y - last_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Record new waypoint if robot has moved far enough
        if (distance >= spacing_)
        {
            addWaypoint(x, y, yaw);
        }
    }

    void addWaypoint(double x, double y, double yaw)
    {
        Waypoint waypoint;

        waypoint.x = x;
        waypoint.y = y;
        waypoint.yaw = yaw;

        waypoints_.push_back(waypoint);

        last_x_ = x;
        last_y_ = y;

        RCLCPP_INFO(this->get_logger(), "Waypoint %zu: x=%.3f, y=%.3f, yaw=%.3f", waypoints_.size(), x, y, yaw);
    }

    double getYaw(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp =2.0 * (q.w * q.z + q.x * q.y);

        double cosy_cosp =1.0 - 2.0 * (q.y * q.y + q.z * q.z);

        return std::atan2(siny_cosp, cosy_cosp);
    }

    void savePath()
    {
        if (waypoints_.empty())
        {
            RCLCPP_WARN(this->get_logger(),"No waypoints recorded. Nothing to save.");
            return;
        }

        std::ofstream file(save_file_);

        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", save_file_.c_str());
            return;
        }

        file << "waypoints:\n";

        for (const auto &waypoint : waypoints_)
        {
            file << "  - x: "
                 << waypoint.x
                 << "\n";

            file << "    y: "
                 << waypoint.y
                 << "\n";

            file << "    yaw: "
                 << waypoint.yaw
                 << "\n";
        }

        file.close();

        RCLCPP_INFO(
            this->get_logger(),
            "Saved %zu waypoints to %s",
            waypoints_.size(),
            save_file_.c_str()
        );
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PathRecorder>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

