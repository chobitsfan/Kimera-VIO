#include "Ros2Visualizer.h"

// Ros2Visualizer::Ros2Visualizer(const VIO::VioParams& vio_params, std::shared_ptr<KimeraRos2NodeCommon> ros2_node) : Visualizer3D(vio_params), ros2_node_(ros2_node) {
// Ros2Visualizer::Ros2Visualizer(const VIO::VioParams& vio_params, std::shared_ptr<KimeraRos2Node> ros2_node) : Visualizer3D(VIO::VisualizationType::kNone), ros2_node_(ros2_node) {
Ros2Visualizer::Ros2Visualizer( std::shared_ptr<KimeraRos2NodeCommon> ros2_node) : Visualizer3D(VIO::VisualizationType::kNone), ros2_node_(ros2_node) {
}

VIO::VisualizerOutput::UniquePtr Ros2Visualizer::spinOnce(const VIO::VisualizerInput& viz_input) {
    if (viz_input.backend_output_) {
        const gtsam::Pose3& pose = viz_input.backend_output_->W_State_Blkf_.pose_;
        const gtsam::Rot3& rotation = pose.rotation();
        const gtsam::Quaternion& quaternion = rotation.toQuaternion();
        const gtsam::Vector3& velocity = viz_input.backend_output_->W_State_Blkf_.velocity_;
        nav_msgs::msg::Odometry odo_msg;
        odo_msg.header.stamp = ros2_node_->get_clock()->now();
        odo_msg.header.frame_id = "map";
        odo_msg.child_frame_id = "map";
        odo_msg.pose.pose.position.x = pose.x();
        odo_msg.pose.pose.position.y = pose.y();
        odo_msg.pose.pose.position.z = pose.z();
        odo_msg.pose.pose.orientation.x = quaternion.x();
        odo_msg.pose.pose.orientation.y = quaternion.y();
        odo_msg.pose.pose.orientation.z = quaternion.z();
        odo_msg.pose.pose.orientation.w = quaternion.w();
        ros2_node_->getOdoPub()->publish(odo_msg);
        // ros2_node_->odo_pub->publish(odo_msg);
    }

    // Return empty output, since in ROS, we only publish, not display...
    return std::make_unique<VIO::VisualizerOutput>();
}
