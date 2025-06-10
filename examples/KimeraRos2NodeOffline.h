// inherit the KimeraRos2NodeCommon class in the node classes 
#pragma once

#include "KimeraRos2NodeCommon.h"
// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/image.hpp"

class KimeraRos2NodeOffline : public KimeraRos2NodeCommon {
    public:
        KimeraRos2NodeOffline();
        // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;
        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr getOdoPub() const override {
            return odo_pub;
        }
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getImgPub() const override {
            return img_pub;
        }
        // rclcpp::Publisher<sensor_msgs::msg::Odometry>::SharedPtr getOdoPub() const override {
        //     return odo_pub;
        // }
        // rclcpp::Publisher<nav_msgs::msg::Image>::SharedPtr getImgPub() const override {
        //     return img_pub;
        // }
    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
};
