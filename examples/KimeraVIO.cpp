/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   KimeraVIO.cpp
 * @brief  Example of VIO pipeline.
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <future>
#include <memory>
#include <utility>

#include "std_msgs/msg/header.hpp"

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/dataprovider/KittiDataProvider.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "KimeraRos2Node.h"
#include "Ros2Visualizer.h"

DEFINE_int32(dataset_type,
             0,
             "Type of parser to use:\n "
             "0: Euroc \n 1: Kitti (not supported).");
DEFINE_string(
    params_folder_path,
    "../params/Euroc",
    "Path to the folder containing the yaml files with the VIO parameters.");

class Ros2Display : public VIO::DisplayBase {
    public:
        KIMERA_POINTER_TYPEDEFS(Ros2Display);
        KIMERA_DELETE_COPY_CONSTRUCTORS(Ros2Display);
        Ros2Display(std::shared_ptr<KimeraRos2Node> ros2_node) : VIO::DisplayBase(VIO::DisplayType::kOpenCV), ros2_node_(ros2_node) {}
        virtual ~Ros2Display() = default;
        /**
        * @brief spinOnce
        * Spins the display once to render the visualizer output.
        * @param viz_output
        */
        void spinOnce(VIO::DisplayInputBase::UniquePtr&& viz_output) override {
            /*std_msgs::msg::Header header;
            header.stamp = ros2_node_->get_clock()->now();
            header.frame_id = "map";
            for (const VIO::ImageToDisplay& img_to_display : viz_output->images_to_display_) {
                std::shared_ptr<sensor_msgs::msg::Image> img_msg = cv_bridge::CvImage(header, "bgr8", img_to_display.image_).toImageMsg();
                ros2_node_->img_pub->publish(*img_msg);
            }*/
            sensor_msgs::msg::Image img_msg;
            img_msg.header.stamp = ros2_node_->get_clock()->now();
            img_msg.header.frame_id = "map";
            img_msg.encoding = "bgr8";
            img_msg.is_bigendian = false;
            for (const VIO::ImageToDisplay& img_to_display : viz_output->images_to_display_) {
                cv::Mat img = img_to_display.image_;
                img_msg.height = img.rows;
                img_msg.width = img.cols;
                img_msg.step = img.step;
                img_msg.data.assign(img.datastart, img.dataend);
                ros2_node_->img_pub->publish(img_msg);
            }
        }
    private:
        std::shared_ptr<KimeraRos2Node> ros2_node_;
};

KimeraRos2Node::KimeraRos2Node(const VIO::VioParams& vio_params) : Node("kimera_vio") , vio_params_(vio_params) {
    frame_count_ = 0;
    odo_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1).best_effort().durability_volatile());
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("tracking", rclcpp::QoS(1).best_effort().durability_volatile());
}

void KimeraRos2Node::init_sub(VIO::Pipeline::Ptr vio_pipeline) {
    auto l_img_cb = [this](sensor_msgs::msg::Image::UniquePtr msg) -> void {
        cv::Mat mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);
        int64_t ts = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
        vio_pipeline_->fillLeftFrameQueue(std::make_unique<VIO::Frame>(frame_count_, ts, vio_params_.camera_params_.at(0), mat.clone()));
        frame_count_++;
    };
    auto imu_cb = [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
        int64_t ts = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
        VIO::ImuAccGyr imu_accgyr;
        imu_accgyr(0) = msg->linear_acceleration.x;
        imu_accgyr(1) = -msg->linear_acceleration.y;
        imu_accgyr(2) = -msg->linear_acceleration.z;
        imu_accgyr(3) = msg->angular_velocity.x;
        imu_accgyr(4) = -msg->angular_velocity.y;
        imu_accgyr(5) = -msg->angular_velocity.z;
        vio_pipeline_->fillSingleImuQueue(VIO::ImuMeasurement(ts, imu_accgyr));
    };
    vio_pipeline_ = vio_pipeline;
    l_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("mono_left", rclcpp::QoS(2).best_effort().durability_volatile(), l_img_cb);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(20).best_effort().durability_volatile(), imu_cb);
}

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse VIO parameters from gflags.
  VIO::VioParams vio_params(FLAGS_params_folder_path);

  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<KimeraRos2Node>(vio_params);

  auto visualizer_ = std::make_unique<Ros2Visualizer>(vio_params, ros_node);
  auto display_ = std::make_unique<Ros2Display>(ros_node);

  // Build dataset parser.
  VIO::DataProviderInterface::Ptr dataset_parser = nullptr;
  switch (FLAGS_dataset_type) {
    case 0: {
      switch (vio_params.frontend_type_) {
        case VIO::FrontendType::kMonoImu: {
        } break;
        case VIO::FrontendType::kStereoImu: {
          dataset_parser = std::make_unique<VIO::EurocDataProvider>(vio_params);
        } break;
        default: {
          LOG(FATAL) << "Unrecognized Frontend type: "
                     << VIO::to_underlying(vio_params.frontend_type_)
                     << ". 0: Mono, 1: Stereo.";
        }
      }
    } break;
    case 1: {
      dataset_parser = std::make_unique<VIO::KittiDataProvider>();
    } break;
    default: {
      LOG(FATAL) << "Unrecognized dataset type: " << FLAGS_dataset_type << "."
                 << " 0: EuRoC, 1: Kitti.";
    }
  }

  VIO::Pipeline::Ptr vio_pipeline;

  switch (vio_params.frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline = std::make_unique<VIO::MonoImuPipeline>(vio_params, std::move(visualizer_), std::move(display_));
      ros_node->init_sub(vio_pipeline);
    } break;
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline = std::make_unique<VIO::StereoImuPipeline>(vio_params, std::move(visualizer_), std::move(display_));
    } break;
    default: {
      LOG(FATAL) << "Unrecognized Frontend type: "
                 << VIO::to_underlying(vio_params.frontend_type_)
                 << ". 0: Mono, 1: Stereo.";
    } break;
  }

  if (vio_params.frontend_type_ == VIO::FrontendType::kStereoImu) {
    auto stereo_pipeline =
        std::dynamic_pointer_cast<VIO::StereoImuPipeline>(vio_pipeline);
    CHECK(stereo_pipeline);

    dataset_parser->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  stereo_pipeline,
                  std::placeholders::_1));
  }

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params.parallel_run_) {
    auto handle = std::async(
        std::launch::async, &VIO::DataProviderInterface::spin, dataset_parser);
    auto handle_pipeline =
        std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline);
    auto handle_shutdown = std::async(
        std::launch::async,
        &VIO::Pipeline::waitForShutdown,
        vio_pipeline,
        [&dataset_parser]() -> bool { return !dataset_parser->hasData(); },
        500,
        true);
    vio_pipeline->spinViz();
    is_pipeline_successful = !handle.get();
    handle_shutdown.get();
    handle_pipeline.get();
  } else {
    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        if (!vio_pipeline->spin()) break;
    };
    vio_pipeline->shutdown();
    is_pipeline_successful = true;
  }

  // Output stats.
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");

  if (is_pipeline_successful) {
    // Log overall time of pipeline run.
    VIO::PipelineLogger logger;
    logger.logPipelineOverallTiming(spin_duration);
  }

  rclcpp::shutdown();

  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
