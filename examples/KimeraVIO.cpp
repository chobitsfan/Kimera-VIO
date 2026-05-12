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
#include <turbojpeg.h>

#include <chrono>
#include <future>
#include <memory>
#include <utility>
#include <sys/mman.h>
#include <fcntl.h>

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

#define MY_ROS_JPG_BUF_SZ (50 * 1024)

using namespace std::literals::chrono_literals;

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
        Ros2Display(std::shared_ptr<KimeraRos2Node> ros2_node) : VIO::DisplayBase(VIO::DisplayType::kOpenCV), ros2_node_(ros2_node) {
            jpg_compressor = tjInitCompress();
            jpg_buf = tjAlloc(MY_ROS_JPG_BUF_SZ);
        }
        ~Ros2Display() {
            tjFree(jpg_buf);
            tjDestroy(jpg_compressor);
        }
        /**
        * @brief spinOnce
        * Spins the display once to render the visualizer output.
        * @param viz_output
        */
        void spinOnce(VIO::DisplayInputBase::UniquePtr&& viz_output) override {
            for (const VIO::ImageToDisplay& img_to_display : viz_output->images_to_display_) {
                if (img_to_display.name_ == "feature_tracks") {
                    /*sensor_msgs::msg::Image img_msg;
                    img_msg.header.stamp = ros2_node_->get_clock()->now();
                    img_msg.header.frame_id = "map";
                    img_msg.encoding = "bgr8";
                    img_msg.is_bigendian = false;
                    img_msg.height = img_to_display.image_.rows;
                    img_msg.width = img_to_display.image_.cols;
                    img_msg.step = img_to_display.image_.step;
                    img_msg.data.assign(img_to_display.image_.datastart, img_to_display.image_.dataend);
                    ros2_node_->img_pub->publish(img_msg);*/
                    unsigned long jpg_sz = MY_ROS_JPG_BUF_SZ;
                    if (tjCompress2(jpg_compressor, img_to_display.image_.data, img_to_display.image_.cols, img_to_display.image_.step, img_to_display.image_.rows, TJPF_BGR, &jpg_buf, &jpg_sz, TJSAMP_420, 20, TJFLAG_FASTDCT) == 0) {
                        sensor_msgs::msg::CompressedImage img_msg;
                        img_msg.header.stamp = ros2_node_->get_clock()->now();
                        img_msg.header.frame_id = "body";
                        img_msg.format = "jpeg";
                        img_msg.data.assign(jpg_buf, jpg_buf + jpg_sz);
                        ros2_node_->img_pub->publish(img_msg);
                    }
                }
            }
        }
    private:
        std::shared_ptr<KimeraRos2Node> ros2_node_;
        tjhandle jpg_compressor;
        unsigned char* jpg_buf;
};

class MyLogSink: public google::LogSink {
    public:
        MyLogSink(const rclcpp::Logger &logger) : logger_(logger) {}
        void send(google::LogSeverity severity, const char* full_filename, const char* base_filename, int line, const google::LogMessageTime& time, const char* message, size_t message_len) override {
            switch (severity) {
                case google::GLOG_WARNING:
                    RCLCPP_WARN_STREAM(logger_, message);
                    break;
                case google::GLOG_ERROR:
                    RCLCPP_ERROR_STREAM(logger_, message);
                    break;
                case google::GLOG_FATAL:
                    RCLCPP_FATAL_STREAM(logger_, message);
                    break;
                case google::GLOG_INFO:
                default:
                    RCLCPP_INFO_STREAM(logger_, message);
                    break;
            }
        }
    private:
        rclcpp::Logger logger_;
};

KimeraRos2Node::KimeraRos2Node(const VIO::VioParams& vio_params) : Node("kimera_vio"), vio_params_(vio_params) {
    frame_count_ = 0;
    pico_pi_t_offset = 0;
    odo_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1).best_effort().durability_volatile());
    img_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("tracking/compressed", rclcpp::QoS(1).best_effort().durability_volatile());
    clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
}

KimeraRos2Node::~KimeraRos2Node() {
}

void KimeraRos2Node::init_sub(VIO::Pipeline::Ptr vio_pipeline) {
    auto l_img_cb = [this](sensor_msgs::msg::Image::UniquePtr msg) -> void {
        int64_t ts = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
        cv::Mat mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);
        clahe_->apply(mat, clahe_dst_);
        vio_pipeline_->fillLeftFrameQueue(std::make_unique<VIO::Frame>(frame_count_, ts, vio_params_.camera_params_.at(0), clahe_dst_.clone()));
        frame_count_++;
    };
    auto imu_cb = [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
        int64_t ts = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
        VIO::ImuAccGyr imu_accgyr;
        imu_accgyr(0) = msg->linear_acceleration.x;
        imu_accgyr(1) = msg->linear_acceleration.y;
        imu_accgyr(2) = msg->linear_acceleration.z;
        imu_accgyr(3) = msg->angular_velocity.x;
        imu_accgyr(4) = msg->angular_velocity.y;
        imu_accgyr(5) = msg->angular_velocity.z;
        vio_pipeline_->fillSingleImuQueue(VIO::ImuMeasurement(ts, imu_accgyr));
    };
    auto t_offset_cb = [this](std_msgs::msg::Int64::UniquePtr msg) -> void {
        pico_pi_t_offset = msg->data;
    };
    vio_pipeline_ = vio_pipeline;
    l_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("mono_left", rclcpp::QoS(1).best_effort().durability_volatile(), l_img_cb);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(200).durability_volatile(), imu_cb);
    t_offset_sub_ = this->create_subscription<std_msgs::msg::Int64>("pico_pi_t_offset", rclcpp::QoS(1).best_effort().durability_volatile(), t_offset_cb);
}

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse VIO parameters from gflags.
  VIO::VioParams vio_params(FLAGS_params_folder_path);

  if (vio_params.backend_params_->autoInitialize_ == 0) {
    double bias[6];
    FILE* file_ptr = fopen("/tmp/imu_bias", "rb");
    fread(bias, sizeof(double), 6, file_ptr);
    fclose(file_ptr);
    gtsam::imuBias::ConstantBias imu_bias = gtsam::imuBias::ConstantBias(Eigen::Map<gtsam::Vector6>(bias));
#if 0
    int shm_fd = shm_open("pos_v_ned", O_RDONLY, 0666);
    float* shm_ptr = (float*)mmap(0, 10*sizeof(float), PROT_READ, MAP_SHARED, shm_fd, 0);
    gtsam::Pose3 pose(gtsam::Rot3::Quaternion(shm_ptr[0], shm_ptr[2], -shm_ptr[3], -shm_ptr[1]), gtsam::Point3(shm_ptr[5], -shm_ptr[6], -shm_ptr[4]));
    gtsam::Vector3 v(shm_ptr[8], -shm_ptr[9], -shm_ptr[7]);
    munmap(shm_ptr, 10*sizeof(float));
    close(shm_fd);
    VIO::VioNavState nav_state(pose, v, imu_bias);
#else
    int shm_fd = shm_open("pos_v_ned", O_RDONLY, 0666);
    float* shm_ptr = (float*)mmap(0, 10*sizeof(float), PROT_READ, MAP_SHARED, shm_fd, 0);
    gtsam::Pose3 fc_pose(gtsam::Rot3::Quaternion(shm_ptr[0], shm_ptr[2], -shm_ptr[3], -shm_ptr[1]), gtsam::Point3(0, 0, 0));
    munmap(shm_ptr, 10*sizeof(float));
    close(shm_fd);
    VIO::VioNavState nav_state(fc_pose, gtsam::Vector3::Zero(), imu_bias);
#endif

    vio_params.backend_params_->initial_ground_truth_state_ = nav_state;
  }

  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<KimeraRos2Node>(vio_params);

  MyLogSink my_log_sink{ros_node->get_logger()};
  google::AddLogSink(&my_log_sink);

  auto visualizer_ = std::make_unique<Ros2Visualizer>(ros_node);
  auto display_ = std::make_unique<Ros2Display>(ros_node);

  // Build dataset parser.
  VIO::DataProviderInterface::Ptr dataset_parser = nullptr;
  switch (FLAGS_dataset_type) {
    case 0: {
      switch (vio_params.frontend_type_) {
        case VIO::FrontendType::kMonoImu: {
          dataset_parser = std::make_unique<VIO::MonoEurocDataProvider>(vio_params);
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
    case 2: // get data from ros2
      break;
    default: {
      LOG(FATAL) << "Unrecognized dataset type: " << FLAGS_dataset_type << "."
                 << " 0: EuRoC, 1: Kitti.";
    }
  }

  VIO::Pipeline::Ptr vio_pipeline;

  switch (vio_params.frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline = std::make_unique<VIO::MonoImuPipeline>(vio_params, std::move(visualizer_), std::move(display_));
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

    if (dataset_parser == nullptr)
        LOG(FATAL) << "ros2 input do not support stereo yet";
    else
        dataset_parser->registerRightFrameCallback(std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue, stereo_pipeline, std::placeholders::_1));
  }

  if (dataset_parser != nullptr) {
      // Register callback to shutdown data provider in case VIO pipeline
      // shutsdown.
      vio_pipeline->registerShutdownCallback(
          std::bind(&VIO::DataProviderInterface::shutdown, dataset_parser));

      // Register callback to vio pipeline.
      dataset_parser->registerImuSingleCallback(std::bind(
          &VIO::Pipeline::fillSingleImuQueue, vio_pipeline, std::placeholders::_1));
      // We use blocking variants to avoid overgrowing the input queues (use
      // the non-blocking versions with real sensor streams)
      dataset_parser->registerLeftFrameCallback(std::bind(
          &VIO::Pipeline::fillLeftFrameQueue, vio_pipeline, std::placeholders::_1));

      if (vio_params.frontend_type_ == VIO::FrontendType::kStereoImu) {
        auto stereo_pipeline =
            std::dynamic_pointer_cast<VIO::StereoImuPipeline>(vio_pipeline);
        CHECK(stereo_pipeline);

        dataset_parser->registerRightFrameCallback(
            std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                      stereo_pipeline,
                      std::placeholders::_1));
      }
  } else ros_node->init_sub(vio_pipeline);

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params.parallel_run_) {
    if (dataset_parser == nullptr) {
        auto handle_pipeline = std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline);
        auto handle_viz = std::async(std::launch::async, &VIO::Pipeline::spinViz, vio_pipeline);
        rclcpp::spin(ros_node);
        vio_pipeline->shutdown();
        handle_pipeline.get();
        handle_viz.get();
        is_pipeline_successful = true;
    } else {
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
    }
  } else {
    while (rclcpp::ok()) {
        if (dataset_parser == nullptr)
            rclcpp::spin_some(ros_node);
        else
            if (!dataset_parser->spin()) break;
        if (!vio_pipeline->spin()) break;
        std::this_thread::sleep_for(1ms);
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
