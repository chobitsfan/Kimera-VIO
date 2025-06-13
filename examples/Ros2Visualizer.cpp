#include "Ros2Visualizer.h"

Ros2Visualizer::Ros2Visualizer(std::shared_ptr<KimeraRos2Node> ros2_node) : Visualizer3D(VIO::VisualizationType::kNone), ros2_node_(ros2_node) {
}

VIO::VisualizerOutput::UniquePtr Ros2Visualizer::spinOnce(const VIO::VisualizerInput& viz_input) {
    if (viz_input.backend_output_) {
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        int latency_ms = (now_ns - viz_input.backend_output_->timestamp_) / 1000000;
        VIO::utils::StatsCollector timing_stats("latency [ms]");
        timing_stats.AddSample(latency_ms);
        const gtsam::Pose3& pose = viz_input.backend_output_->W_State_Blkf_.pose_;
        const gtsam::Rot3& rotation = pose.rotation();
        const gtsam::Quaternion& quaternion = rotation.toQuaternion();
        const gtsam::Vector3& velocity = viz_input.backend_output_->W_State_Blkf_.velocity_;
        nav_msgs::msg::Odometry odo_msg;
        odo_msg.header.stamp = ros2_node_->get_clock()->now();
        odo_msg.header.frame_id = "map";
        odo_msg.child_frame_id = "map";
        odo_msg.pose.pose.position.x = -pose.z();
        odo_msg.pose.pose.position.y = -pose.x();
        odo_msg.pose.pose.position.z = pose.y();
        odo_msg.pose.pose.orientation.x = -quaternion.z();
        odo_msg.pose.pose.orientation.y = -quaternion.x();
        odo_msg.pose.pose.orientation.z = quaternion.y();
        odo_msg.pose.pose.orientation.w = quaternion.w();
        ros2_node_->odo_pub->publish(odo_msg);
    }

    // Return empty output, since in ROS, we only publish, not display...
    return std::make_unique<VIO::VisualizerOutput>();
}
