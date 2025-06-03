#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class KimeraRos2Node : public rclcpp::Node {
    public:
        KimeraRos2Node();
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;
};
