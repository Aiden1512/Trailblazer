
#include <chrono>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class HuskyWASD : public rclcpp::Node
{
public:
    HuskyWASD() : Node("husky_wasd")
    {
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("/husky1/cmd_vel", 10);
    }

    void run()
    {
        // so that dont need to press enter between keys
        termios old_terminal, new_terminal;
        tcgetattr(STDIN_FILENO, &old_terminal);
        new_terminal = old_terminal;
        new_terminal.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);

        std::cout << "Husky WASD Controller\n"
                  << "W = Forward | S = Backward\n"
                  << "A = Left    | D = Right\n"
                  << "X = Stop    | Q = Quit\n";

        char key;
        bool running = true;

        while (rclcpp::ok() && running)
        {
            std::cin.get(key);

            geometry_msgs::msg::Twist msg;

            switch (key)
            {
                case 'w':
                case 'W':
                    msg.linear.x = 0.5;
                    break;

                case 's':
                case 'S':
                    msg.linear.x = -0.5;
                    break;

                case 'a':
                case 'A':
                    msg.angular.z = 0.8;
                    break;

                case 'd':
                case 'D':
                    msg.angular.z = -0.8;
                    break;

                case 'x':
                case 'X':
                    // All velocities remain zero
                    break;

                case 'q':
                case 'Q':
                    running = false;
                    break;

                default:
                    continue;
            }

            publisher_->publish(msg);
            rclcpp::spin_some(shared_from_this());

        }

        publisher_->publish(geometry_msgs::msg::Twist{});

        // terminal back to normal
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
        std::cout << "\nController stopped.\n";
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HuskyWASD>();
    node->run();

    rclcpp::shutdown();
    return 0;
}
