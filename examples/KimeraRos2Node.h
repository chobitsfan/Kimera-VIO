#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

class KimeraRos2Node : public rclcpp::Node {
    public:
        KimeraRos2Node(const VIO::VioParams& vio_params);
        ~KimeraRos2Node();
        void init_sub(VIO::Pipeline::Ptr vio_pipeline);
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;
        cv::Mat img_to_pub_;
        std::atomic<bool> img_avail_;
    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr l_img_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        VIO::Pipeline::Ptr vio_pipeline_;
        VIO::VioParams vio_params_;
        uint64_t frame_count_;
        std::thread img_pub_thread_;
        std::atomic<bool> thread_go_;
        void pub_worker();
};
