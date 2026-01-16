#include "Ros2Visualizer.h"

Ros2Visualizer::Ros2Visualizer(std::shared_ptr<KimeraRos2Node> ros2_node) : Visualizer3D(VIO::VisualizationType::kNone), ros2_node_(ros2_node) {
}

VIO::VisualizerOutput::UniquePtr Ros2Visualizer::spinOnce(const VIO::VisualizerInput& viz_input) {
    if (viz_input.backend_output_) {
        if (ros2_node_->pico_pi_t_offset != 0) {
            int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            int latency_ms = (now_ns + ros2_node_->pico_pi_t_offset - viz_input.backend_output_->timestamp_) / 1000000;
            VIO::utils::StatsCollector timing_stats("latency [ms]");
            timing_stats.AddSample(latency_ms);
        }
        const gtsam::Pose3& pose = viz_input.backend_output_->W_State_Blkf_.pose_;
        const gtsam::Rot3& rotation = pose.rotation();
        const gtsam::Quaternion& quaternion = rotation.toQuaternion();
        const gtsam::Vector3& velocity = viz_input.backend_output_->W_State_Blkf_.velocity_;
        const gtsam::Matrix& cov = viz_input.backend_output_->state_covariance_lkf_.block(0, 0, 9, 9);
        nav_msgs::msg::Odometry odo_msg;
        odo_msg.header.stamp.sec = static_cast<int32_t>(viz_input.backend_output_->timestamp_ / 1000000000);
        odo_msg.header.stamp.nanosec = static_cast<uint32_t>(viz_input.backend_output_->timestamp_ % 1000000000);
        odo_msg.header.frame_id = "map";
        odo_msg.child_frame_id = "map";
        odo_msg.pose.pose.position.x = -pose.z();
        odo_msg.pose.pose.position.y = -pose.x();
        odo_msg.pose.pose.position.z = pose.y();
        odo_msg.pose.pose.orientation.x = -quaternion.z();
        odo_msg.pose.pose.orientation.y = -quaternion.x();
        odo_msg.pose.pose.orientation.z = quaternion.y();
        odo_msg.pose.pose.orientation.w = quaternion.w();
        odo_msg.twist.twist.linear.x = -velocity(2);
        odo_msg.twist.twist.linear.y = -velocity(0);
        odo_msg.twist.twist.linear.z = velocity(1);
        ros2_node_->odo_pub->publish(odo_msg);

        float uncertainty = sqrtf(cov(3, 3) + cov(4, 4) + cov(5, 5));
        std_msgs::msg::Float32 uncertainty_msg;
        uncertainty_msg.data = uncertainty;
        ros2_node_->transl_uncertainty_pub->publish(uncertainty_msg);
        uncertainty = sqrtf(cov(6, 6) + cov(7, 7) + cov(8, 8));
        uncertainty_msg.data = uncertainty;
        ros2_node_->speed_uncertainty_pub->publish(uncertainty_msg);
    }

    // Return empty output, since in ROS, we only publish, not display...
    return std::make_unique<VIO::VisualizerOutput>();
}
