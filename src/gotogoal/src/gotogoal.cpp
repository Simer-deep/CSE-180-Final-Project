#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher;

bool initialized = false;
bool reached = false;

double goalX = 0.0;
double goalY = 0.0;

void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double theta = atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    );

    if (initialized == false)
    {
        double a = 1.0;
        double b = 0.5;

        goalX = x + a * cos(theta) - b * sin(theta);
        goalY = y + a * sin(theta) + b * cos(theta);

        RCLCPP_INFO(
            rclcpp::get_logger("gotogoal"),
            "Goal in odom: x = %.3f, y = %.3f",
            goalX,
            goalY
        );

        initialized = true;
    }

    double dx = goalX - x;
    double dy = goalY - y;

    double xR = cos(theta) * dx + sin(theta) * dy;
    double yR = -sin(theta) * dx + cos(theta) * dy;

    double distance = sqrt(xR * xR + yR * yR);
    double alpha = atan2(yR, xR);

    geometry_msgs::msg::Twist vel;

    if (distance < 0.10)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;

        if (reached == false)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("gotogoal"),
                "Goal reached"
            );

            reached = true;
        }
    }
    else
    {
        double turnSpeed = alpha;

        if (turnSpeed > 0.8)
            turnSpeed = 0.8;

        if (turnSpeed < -0.8)
            turnSpeed = -0.8;

        vel.angular.z = turnSpeed;

        if (fabs(alpha) < 0.3)
        {
            double forwardSpeed = 0.5 * distance;

            if (forwardSpeed > 0.2)
                forwardSpeed = 0.2;

            vel.linear.x = forwardSpeed;
        }
        else
        {
            vel.linear.x = 0.0;
        }
    }

    velPublisher->publish(vel);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("gotogoal");

    velPublisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    auto odomSubscriber =
        node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            odomCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
