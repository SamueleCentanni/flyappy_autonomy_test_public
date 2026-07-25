#include "flyappy/flyappy_ros.hpp"

#include <array>

namespace flyappy
{

inline constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(rclcpp::Node::SharedPtr node) : node_(node)
{
    pub_acceleration_command_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
            "/flyappy_acc", QUEUE_SIZE);
    sub_velocity_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/flyappy_vel", QUEUE_SIZE,
            std::bind(&FlyappyRos::velocityCallback, this, std::placeholders::_1));
    sub_laser_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/flyappy_laser_scan", QUEUE_SIZE,
            std::bind(&FlyappyRos::laserScanCallback, this, std::placeholders::_1));
    sub_game_ended_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/flyappy_game_ended", QUEUE_SIZE,
            std::bind(&FlyappyRos::gameEndedCallback, this, std::placeholders::_1));
}

void FlyappyRos::velocityCallback(const geometry_msgs::msg::Vector3& msg)
{
    const auto cmd = flyappy_.computeAcceleration(msg.x, msg.y);

    geometry_msgs::msg::Vector3 acc_cmd;
    acc_cmd.x = cmd.ax;
    acc_cmd.y = cmd.ay;
    pub_acceleration_command_->publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::msg::LaserScan& msg)
{
    if (msg.ranges.size() != flyappy::kNumRays)
    {
        RCLCPP_WARN(node_->get_logger(), "Unexpected number of laser rays: %zu",
                    msg.ranges.size());
        return;
    }

    std::array<double, flyappy::kNumRays> ranges{};
    for (size_t i = 0; i < flyappy::kNumRays; ++i)
    {
        ranges[i] = static_cast<double>(msg.ranges[i]);
    }

    flyappy_.setLaserScan(ranges);
}

void FlyappyRos::gameEndedCallback(const std_msgs::msg::Bool& msg)
{
    if (msg.data)
    {
        RCLCPP_INFO(node_->get_logger(), "Crash detected.");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "End of countdown.");
    }

    flyappy_.reset();
}

}  // namespace flyappy
