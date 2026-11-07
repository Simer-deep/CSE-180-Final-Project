#include <memory>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

const double v_max = 0.10;
const double w_max = 0.40;
const double timeout = 0.50;

rclcpp::Node::SharedPtr node;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

geometry_msgs::msg::Twist last_command;
rclcpp::Time last_command_time;

bool received_command = false;
bool stopped_by_timeout = false;

void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::Twist safe_command;

    safe_command.linear.x = std::max(-v_max,
                             std::min(v_max, msg->linear.x));

    safe_command.angular.z = std::max(-w_max,
                              std::min(w_max, msg->angular.z));

    last_command = safe_command;
    last_command_time = node->now();
    received_command = true;

    if (stopped_by_timeout)
    {
        RCLCPP_INFO(node->get_logger(), "Driving resumed");
        stopped_by_timeout = false;
    }
}

void timer_callback()
{
    geometry_msgs::msg::Twist output;

    if (received_command)
    {
        double elapsed = (node->now() - last_command_time).seconds();

        if (elapsed > timeout)
        {
            if (!stopped_by_timeout)
            {
                RCLCPP_INFO(node->get_logger(), "Robot stopped: command timeout");
                stopped_by_timeout = true;
            }
        }
        else
        {
            output = last_command;
        }
    }

    publisher->publish(output);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("safedrive");

    publisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    auto subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10, command_callback);

    auto timer = node->create_wall_timer(100ms, timer_callback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
