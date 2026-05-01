/**
 * @file   KimeraVioRos.cpp
 * @brief  ROS 2 wrapper around Kimera-VIO Pipeline.
 *
 * Slice A scope: stereo+IMU online provider, minimal RosVisualizer.
 */

#include "kimera_vio_ros/KimeraVioRos.hpp"

#include <future>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <kimera-vio/pipeline/MonoImuPipeline.h>
#include <kimera-vio/pipeline/RgbdImuPipeline.h>
#include <kimera-vio/pipeline/StereoImuPipeline.h>
#include <kimera-vio/utils/Timer.h>

#include "kimera_vio_ros/RosOnlineDataProvider.hpp"

namespace VIO {

namespace {

template <typename T>
T getOrDeclare(rclcpp::Node* node, const std::string& name, const T& def) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

template <typename T>
T getRequired(rclcpp::Node* node, const std::string& name) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name);
  }
  T value;
  CHECK(node->get_parameter(name, value)) << "Required parameter '" << name
                                          << "' not set.";
  return value;
}

}  // namespace

KimeraVioRos::KimeraVioRos(const rclcpp::NodeOptions& opts)
    : node_(std::make_shared<rclcpp::Node>("kimera_vio_ros", opts)),
      vio_params_(nullptr),
      vio_pipeline_(nullptr),
      use_rviz_(true),
      use_lcd_registration_server_(false),
      data_provider_(nullptr),
      ros_visualizer_(nullptr),
      ros_lcd_visualizer_(nullptr),
      lcd_registration_server_(nullptr),
      restart_vio_pipeline_(false),
      shutting_down_(false) {
  use_rviz_ = getOrDeclare<bool>(node_.get(), "use_rviz", true);
  use_lcd_registration_server_ =
      getOrDeclare<bool>(node_.get(), "use_lcd_registration_server", false);

  const auto params_path =
      getRequired<std::string>(node_.get(), "params_folder_path");
  CHECK(!params_path.empty());
  const auto sensor_params_path =
      getOrDeclare<std::string>(node_.get(), "sensor_params_folder_path", "");

  if (sensor_params_path.empty()) {
    VLOG(1) << "Using single parameter folder: " << params_path;
    vio_params_ = std::make_shared<VioParams>(params_path);
  } else {
    VLOG(1) << "Using split parameter folders: " << params_path
            << " (general), " << sensor_params_path << " (sensor)";
    vio_params_ = std::make_shared<VioParams>(params_path, sensor_params_path);
  }

  restart_vio_pipeline_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/restart_kimera_vio",
      std::bind(&KimeraVioRos::restartCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

KimeraVioRos::~KimeraVioRos() = default;

void KimeraVioRos::shutdown() {
  shutting_down_ = true;
  if (lcd_registration_server_) lcd_registration_server_->stop();
  if (data_provider_) data_provider_->shutdown();
  if (vio_pipeline_ && !vio_pipeline_->isShutdown()) {
    vio_pipeline_->shutdown();
  }
}

RosDataProviderInterface::UniquePtr KimeraVioRos::createDataProvider(
    const VioParams& vio_params) {
  // Slice A: only the online provider exists. Offline runs use `ros2 bag
  // play` to feed the same topics; RosBagDataProvider was deliberately not
  // ported (see CLAUDE.md > Architectural decisions).
  return std::make_unique<RosOnlineDataProvider>(node_.get(), vio_params);
}

bool KimeraVioRos::runKimeraVio() {
  if (use_rviz_) {
    ros_visualizer_.reset();
    ros_display_.reset();
    VLOG(1) << "Creating ROS visualizer + display.";
    CHECK(vio_params_);
    ros_visualizer_ = std::make_unique<RosVisualizer>(node_.get(),
                                                      *vio_params_);
    ros_display_ = std::make_unique<RosDisplay>(node_.get());
  } else {
    ros_visualizer_ = nullptr;
    ros_display_ = nullptr;
  }

  VLOG(1) << "Destroying VIO pipeline.";
  vio_pipeline_.reset();
  VLOG(1) << "Destroying data provider.";
  data_provider_.reset();

  std::unique_ptr<PreloadedVocab> preloaded_vocab;
  if (FLAGS_use_lcd) {
    preloaded_vocab.reset(new PreloadedVocab());
  }

  VLOG(1) << "Creating data provider.";
  data_provider_ = createDataProvider(*vio_params_);
  CHECK(data_provider_);

  VLOG(1) << "Creating Kimera-VIO Pipeline.";
  Visualizer3D::UniquePtr viz_for_pipeline = std::move(ros_visualizer_);
  DisplayBase::UniquePtr display_for_pipeline = std::move(ros_display_);
  switch (vio_params_->frontend_type_) {
    case FrontendType::kStereoImu: {
      vio_pipeline_ = std::make_unique<StereoImuPipeline>(
          *vio_params_, std::move(viz_for_pipeline),
          std::move(display_for_pipeline), std::move(preloaded_vocab));
      break;
    }
    case FrontendType::kMonoImu: {
      vio_pipeline_ = std::make_unique<MonoImuPipeline>(
          *vio_params_, std::move(viz_for_pipeline),
          std::move(display_for_pipeline), std::move(preloaded_vocab));
      break;
    }
    case FrontendType::kRgbdImu: {
      vio_pipeline_ = std::make_unique<RgbdImuPipeline>(
          *vio_params_, std::move(viz_for_pipeline),
          std::move(display_for_pipeline), std::move(preloaded_vocab));
      break;
    }
    default:
      LOG(FATAL) << "Unrecognized frontend type: "
                 << VIO::to_underlying(vio_params_->frontend_type_);
  }
  CHECK(vio_pipeline_);

  // Slice C: wire LCD output and (optionally) the frame-registration service.
  // Both require the pipeline to exist and FLAGS_use_lcd to be set so that
  // Pipeline created an LcdModule.
  if (FLAGS_use_lcd) {
    if (!ros_lcd_visualizer_) {
      ros_lcd_visualizer_ =
          std::make_unique<RosLoopClosureVisualizer>(node_.get());
    }
    vio_pipeline_->registerLcdOutputCallback(
        [this](const LcdOutput::ConstPtr& msg) {
          if (ros_lcd_visualizer_) ros_lcd_visualizer_->publishLcdOutput(msg);
        });
    if (use_lcd_registration_server_) {
      LcdModule* lcd = vio_pipeline_->getLcdModule();
      if (lcd) {
        lcd_registration_server_.reset();
        lcd_registration_server_ =
            std::make_unique<LcdRegistrationServer>(node_.get(), lcd);
      } else {
        LOG(WARNING) << "use_lcd_registration_server requested but pipeline "
                        "did not create an LcdModule.";
      }
    }
  } else if (use_lcd_registration_server_) {
    LOG(WARNING) << "use_lcd_registration_server requires --use_lcd=true; "
                    "service not advertised.";
  }

  VLOG(1) << "Connecting Pipeline + Data Provider.";
  connectVIO();
  return spin();
}

void KimeraVioRos::connectVIO() {
  CHECK(data_provider_);
  CHECK(vio_pipeline_);

  vio_pipeline_->registerShutdownCallback(
      std::bind(&DataProviderInterface::shutdown, data_provider_.get()));

  data_provider_->registerImuSingleCallback(
      std::bind(&Pipeline::fillSingleImuQueue, vio_pipeline_.get(),
                std::placeholders::_1));
  data_provider_->registerImuMultiCallback(
      std::bind(&Pipeline::fillMultiImuQueue, vio_pipeline_.get(),
                std::placeholders::_1));
  data_provider_->registerLeftFrameCallback(
      std::bind(&Pipeline::fillLeftFrameQueue, vio_pipeline_.get(),
                std::placeholders::_1));
  data_provider_->registerExternalOdomCallback(
      std::bind(&Pipeline::fillExternalOdomQueue, vio_pipeline_.get(),
                std::placeholders::_1));

  if (vio_params_->frontend_type_ == FrontendType::kStereoImu) {
    auto* stereo = dynamic_cast<StereoImuPipeline*>(vio_pipeline_.get());
    CHECK(stereo);
    data_provider_->registerRightFrameCallback(
        std::bind(&StereoImuPipeline::fillRightFrameQueue, stereo,
                  std::placeholders::_1));
  }
  if (vio_params_->frontend_type_ == FrontendType::kRgbdImu) {
    auto* rgbd = dynamic_cast<RgbdImuPipeline*>(vio_pipeline_.get());
    CHECK(rgbd);
    data_provider_->registerDepthFrameCallback(
        std::bind(&RgbdImuPipeline::fillDepthFrameQueue, rgbd,
                  std::placeholders::_1));
  }
}

bool KimeraVioRos::spin() {
  CHECK(vio_params_);
  CHECK(vio_pipeline_);
  CHECK(data_provider_);

  auto tic = utils::Timer::tic();
  bool is_pipeline_successful = false;

  if (vio_params_->parallel_run_) {
    // The data provider runs callback-driven on the rclcpp executor; its
    // spin() is a liveness probe. The Pipeline owns its own threads.
    std::future<bool> dp_handle =
        std::async(std::launch::async, &RosDataProviderInterface::spin,
                   data_provider_.get());
    std::future<bool> viz_handle =
        std::async(std::launch::async, &Pipeline::spinViz, vio_pipeline_.get());
    std::future<bool> vio_handle =
        std::async(std::launch::async, &Pipeline::spin, vio_pipeline_.get());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok() && !restart_vio_pipeline_ && !shutting_down_) {
      const auto stats = vio_pipeline_->printStatistics();
      if (!stats.empty()) {
        LOG_EVERY_N(INFO, 20) << stats;
      }
      rate.sleep();
      if (vio_pipeline_->hasFinished() && data_provider_->isShutdown()) {
        break;
      }
    }
    if (!vio_pipeline_->isShutdown()) vio_pipeline_->shutdown();

    LOG(INFO) << "Joining VIO + viz + data-provider threads.";
    vio_handle.get();
    viz_handle.get();
    dp_handle.get();
    is_pipeline_successful = true;

    if (restart_vio_pipeline_) {
      restart_vio_pipeline_ = false;
      LOG(WARNING) << "Restarting Kimera-VIO pipeline.";
      return runKimeraVio();
    }
  } else {
    while (rclcpp::ok() && data_provider_->spin() && vio_pipeline_->spin()) {
      LOG(INFO) << vio_pipeline_->printStatistics();
      vio_pipeline_->spinViz();
    }
    vio_pipeline_->shutdown();
    is_pipeline_successful = true;
  }
  const auto dur = utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << dur.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");
  return is_pipeline_successful;
}

void KimeraVioRos::restartCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  if (!restart_vio_pipeline_) {
    restart_vio_pipeline_ = true;
    res->message = "Kimera-VIO restart requested.";
    res->success = true;
  } else {
    res->message = "Kimera-VIO already restarting.";
    res->success = false;
  }
  LOG(WARNING) << res->message;
}

}  // namespace VIO
