#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher;

bool initialized = false;
bool obstacleDetected = false;

int currentWaypoint = 0;

const double L = 0.80;
const double D_STOP = 0.40;

double waypointSX[4] = {L, L, 0.0, 0.0};
double waypointSY[4] = {0.0, L, L, 0.0};

double waypointX[4];
double waypointY[4];

void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    bool tooClose = false;

    for (double range : msg->ranges)
    {
        if (range >= msg->range_min && range <= msg->range_max && range < D_STOP)
        {
            tooClose = true;
            break;
        }
    }

    if (tooClose && !obstacleDetected)
        RCLCPP_INFO(rclcpp::get_logger("patrol"), "Obstacle detected: pausing");

    if (!tooClose && obstacleDetected)
        RCLCPP_INFO(rclcpp::get_logger("patrol"), "Path clear: resuming");

    obstacleDetected = tooClose;

    if (obstacleDetected)
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        velPublisher->publish(stop);
    }
}

void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double theta = atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz));

    if (initialized == false)
    {
        for (int i = 0; i < 4; i++)
        {
            waypointX[i] = x + waypointSX[i] * cos(theta) - waypointSY[i] * sin(theta);
            waypointY[i] = y + waypointSX[i] * sin(theta) + waypointSY[i] * cos(theta);
        }

        RCLCPP_INFO(rclcpp::get_logger("patrol"),
                    "Waypoint 1 in odom: x = %.3f, y = %.3f",
                    waypointX[0], waypointY[0]);

        RCLCPP_INFO(rclcpp::get_logger("patrol"),
                    "Waypoint 3 in odom: x = %.3f, y = %.3f",
                    waypointX[2], waypointY[2]);

        initialized = true;
    }

    double dx = waypointX[currentWaypoint] - x;
    double dy = waypointY[currentWaypoint] - y;

    double xR = cos(theta) * dx + sin(theta) * dy;
    double yR = -sin(theta) * dx + cos(theta) * dy;

    double distance = sqrt(xR * xR + yR * yR);
    double alpha = atan2(yR, xR);

    geometry_msgs::msg::Twist vel;

    if (obstacleDetected)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }
    else if (distance < 0.10)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("patrol"),
                    "Waypoint %d reached", currentWaypoint + 1);

        currentWaypoint++;

        if (currentWaypoint == 4)
            currentWaypoint = 0;
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("patrol");

    velPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto odomSubscriber = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odomCallback);

    auto scanSubscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), scanCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
