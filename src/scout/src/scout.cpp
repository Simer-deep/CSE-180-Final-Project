#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher;
sensor_msgs::msg::LaserScan::SharedPtr scan;

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

geometry_msgs::msg::PointStamped target;
geometry_msgs::msg::PointStamped home;

enum Phase { DETECT, APPROACH, RETURN, DONE };
Phase phase = DETECT;

bool tfFailed = false;

double parkDistance = 0.30;
double beta = 20.0 * 3.141592653589793 / 180.0;

void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan = msg;
}

void stopRobot()
{
    geometry_msgs::msg::Twist vel;
    velPublisher->publish(vel);
}

void driveTo(geometry_msgs::msg::PointStamped goal, double stopDistance)
{
    auto tf = tfBuffer->lookupTransform(
        "base_link", "odom", tf2::TimePointZero);

    geometry_msgs::msg::PointStamped p;
    tf2::doTransform(goal, p, tf);

    double xR = p.point.x;
    double yR = p.point.y;

    double distance = sqrt(xR * xR + yR * yR);
    double alpha = atan2(yR, xR);

    geometry_msgs::msg::Twist vel;

    if (distance < stopDistance)
    {
        stopRobot();

        if (phase == APPROACH)
        {
            RCLCPP_INFO(node->get_logger(), "Reached parking distance");
            phase = RETURN;
        }
        else if (phase == RETURN)
        {
            RCLCPP_INFO(node->get_logger(), "Returned home");
            phase = DONE;
        }

        return;
    }

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

    velPublisher->publish(vel);
}

void timerCallback()
{
    if (phase == DONE)
    {
        stopRobot();
        return;
    }

    try
    {
        if (phase == DETECT)
        {
            stopRobot();

            if (scan == nullptr)
                return;

            rclcpp::Time scanTime(scan->header.stamp);

            double age = (node->now() - scanTime).seconds();

            if (fabs(age) > 2.0)
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(), *node->get_clock(), 2000,
                    "Scan time does not match node time");
                return;
            }

            auto scanTf = tfBuffer->lookupTransform(
                "base_link", scan->header.frame_id, scanTime);

            auto odomTf = tfBuffer->lookupTransform(
                "odom", "base_link", scanTime);

            double closest = std::numeric_limits<double>::infinity();
            bool found = false;

            geometry_msgs::msg::PointStamped bestScan;
            geometry_msgs::msg::PointStamped bestRobot;

            for (size_t i = 0; i < scan->ranges.size(); i++)
            {
                double r = scan->ranges[i];

                if (!std::isfinite(r))
                    continue;

                if (r < scan->range_min || r > scan->range_max)
                    continue;

                double angle = scan->angle_min +
                               i * scan->angle_increment;

                geometry_msgs::msg::PointStamped pScan;
                pScan.header = scan->header;

                pScan.point.x = r * cos(angle);
                pScan.point.y = r * sin(angle);
                pScan.point.z = 0.0;

                geometry_msgs::msg::PointStamped pRobot;
                tf2::doTransform(pScan, pRobot, scanTf);

                double x = pRobot.point.x;
                double y = pRobot.point.y;

                if (x <= 0.0)
                    continue;

                if (fabs(atan2(y, x)) > beta)
                    continue;

                double distance = sqrt(x * x + y * y);

                if (distance < closest)
                {
                    closest = distance;
                    bestScan = pScan;
                    bestRobot = pRobot;
                    found = true;
                }
            }

            if (tfFailed)
            {
                RCLCPP_WARN(node->get_logger(), "TF2 recovered");
                tfFailed = false;
            }

            if (!found)
            {
                RCLCPP_INFO_THROTTLE(
                    node->get_logger(), *node->get_clock(), 1000,
                    "No object found");
                return;
            }

            tf2::doTransform(bestRobot, target, odomTf);

            geometry_msgs::msg::PointStamped origin;
            origin.header.frame_id = "base_link";
            origin.header.stamp = scan->header.stamp;

            tf2::doTransform(origin, home, odomTf);

            RCLCPP_INFO(node->get_logger(),
                "Target scanner: %.5f %.5f %.5f",
                bestScan.point.x, bestScan.point.y, bestScan.point.z);

            RCLCPP_INFO(node->get_logger(),
                "Target robot: %.5f %.5f %.5f",
                bestRobot.point.x, bestRobot.point.y, bestRobot.point.z);

            RCLCPP_INFO(node->get_logger(),
                "Target odom: %.5f %.5f %.5f",
                target.point.x, target.point.y, target.point.z);

            RCLCPP_INFO(node->get_logger(),
                "Home odom: %.5f %.5f %.5f",
                home.point.x, home.point.y, home.point.z);

            phase = APPROACH;
        }
        else if (phase == APPROACH)
        {
            driveTo(target, parkDistance);
        }
        else if (phase == RETURN)
        {
            driveTo(home, 0.10);
        }

        if (tfFailed)
        {
            RCLCPP_WARN(node->get_logger(), "TF2 recovered");
            tfFailed = false;
        }
    }
    catch (const tf2::TransformException &ex)
    {
        stopRobot();

        if (!tfFailed)
        {
            RCLCPP_WARN(node->get_logger(),
                        "TF2 failed: %s", ex.what());
            tfFailed = true;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("scout");

    tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    velPublisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    auto scanSubscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), scanCallback);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100), timerCallback);

    RCLCPP_INFO(node->get_logger(), "Scout started");

    rclcpp::spin(node);

    stopRobot();
    rclcpp::shutdown();

    return 0;
}
