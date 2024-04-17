//#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>
#include <unistd.h>
#include <iostream>

class TurtleTeleop : public rclcpp::Node {
public:
    TurtleTeleop()
        : Node("turtle_teleop_key"), linear_velocity_(0.5), angular_velocity_(0.5) {
        // Initialize the publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Display control instructions
        std::cout << "Control the turtle using keyboard:" << std::endl;
        std::cout << "Use 'W' for forward, 'S' for backward, 'A' for left, 'D' for right" << std::endl;
        std::cout << "Press 'Q' to quit" << std::endl;

        run_teleop();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_velocity_;
    double angular_velocity_;

    void run_teleop() {
        geometry_msgs::msg::Twist twist;

        while (rclcpp::ok()) {
            char key = get_key();

            if (key == 'q' || key == 'Q') {
                std::cout << "Exiting teleop..." << std::endl;
                break;
            }

            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            switch (key) {
                case 'w':
                case 'W':
                    twist.linear.x = linear_velocity_;
                    break;
                case 's':
                case 'S':
                    twist.linear.x = -linear_velocity_;
                    break;
                case 'a':
                case 'A':
                    twist.angular.z = angular_velocity_;
                    break;
                case 'd':
                case 'D':
                    twist.angular.z = -angular_velocity_;
                    break;
                default:
                    continue;
            }

            // Publish the Twist message
            publisher_->publish(twist);
        }

        // Stop the turtle when quitting
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
    }

    char get_key() {
        // Set up terminal settings for raw input
        termios old_settings, new_settings;
        tcgetattr(STDIN_FILENO, &old_settings);
        new_settings = old_settings;
        cfmakeraw(&new_settings);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

        char key = getchar();

        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
        
        return key;
    }
};

int main(int argc, char **argv) {
    // Initialize rclcpp
    rclcpp::init(argc, argv);
    
    // Create the teleop node
    auto teleop_node = std::make_shared<TurtleTeleop>();

    // Keep the node running
    rclcpp::spin(teleop_node);

    // Shutdown rclcpp
    rclcpp::shutdown();

    return 0;
}
