// shared base interface to both the online and offline versions of the KimeraVIO examples
//
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"

class KimeraRos2NodeCommon : public rclcpp::Node {
    public:
        KimeraRos2NodeCommon(const std::string& node_name)
            : rclcpp::Node(node_name) {}
        /* KimeraRos2NodeCommon(); */
        virtual rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr getOdoPub() const = 0;
        virtual rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getImgPub() const = 0;
        /* rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub; */
        /* rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub; */

        virtual ~KimeraRos2NodeCommon() = default;
};
