#include "rclcpp/rclcpp.hpp"
#include "trail_decision_making/mission_manager.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trail_decision_making::MissionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
