/**
 * @file   RosOnlineDataProvider.cpp
 * @brief  Slice A: live stereo + IMU data provider on rclcpp.
 */

#include "kimera_vio_ros/RosOnlineDataProvider.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>

#include "kimera_vio_ros/utils/UtilsRos.hpp"

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

constexpr int64_t toNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

}  // namespace

RosOnlineDataProvider::RosOnlineDataProvider(rclcpp::Node* node,
                                             const VioParams& vio_params)
    : RosDataProviderInterface(node, vio_params),
      frame_count_(FrameId(0)),
      reinit_flag_(false),
      force_same_image_timestamp_(true),
      use_external_odom_(false) {
  // Frame IDs (required).
  base_link_frame_id_ = getRequired<std::string>(node_, "base_link_frame_id");
  left_cam_frame_id_ = getRequired<std::string>(node_, "left_cam_frame_id");
  right_cam_frame_id_ = getRequired<std::string>(node_, "right_cam_frame_id");

  force_same_image_timestamp_ =
      getOrDeclare<bool>(node_, "force_same_image_timestamp", true);
  use_external_odom_ = getOrDeclare<bool>(node_, "use_external_odom", false);

  // Static TF broadcaster: publish camera extrinsics.
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node_);
  publishStaticTf(vio_params_.camera_params_.at(0).body_Pose_cam_,
                  base_link_frame_id_,
                  left_cam_frame_id_);
  if (vio_params_.camera_params_.size() == 2) {
    publishStaticTf(vio_params_.camera_params_.at(1).body_Pose_cam_,
                    base_link_frame_id_,
                    right_cam_frame_id_);
  }

  // Ground-truth odom subscriber (only if requested).
  if (vio_params_.backend_params_->autoInitialize_ == 0 || log_gt_data_) {
    gt_odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "gt_odom", 10,
        std::bind(&RosOnlineDataProvider::callbackGtOdom, this,
                  std::placeholders::_1));
  }
  waitForGroundTruthPoseIfRequested();

  // Dedicated callback group for IMU (reentrant).
  imu_cb_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions imu_opts;
  imu_opts.callback_group = imu_cb_group_;
  rclcpp::QoS imu_qos = rclcpp::SensorDataQoS().keep_last(1000);
  imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos,
      std::bind(&RosOnlineDataProvider::callbackIMU, this,
                std::placeholders::_1),
      imu_opts);

  // Stereo image subscriptions via image_transport SubscriberFilter.
  // image_transport in ROS 2 needs a NodeHandle-equivalent. We borrow the
  // node's shared pointer through a non-owning instance.
  it_ = std::make_unique<image_transport::ImageTransport>(
      rclcpp::Node::SharedPtr(node_, [](rclcpp::Node*) {}));

  switch (vio_params_.frontend_type_) {
    case FrontendType::kStereoImu: subscribeStereo(); break;
    case FrontendType::kMonoImu:   subscribeMono();   break;
    case FrontendType::kRgbdImu:   subscribeRgbd();   break;
    default:
      LOG(FATAL) << "Unrecognised frontend type "
                 << VIO::to_underlying(vio_params_.frontend_type_);
  }

  // Reinit subscribers (one-shot trigger + pose).
  reinit_flag_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
      "reinit_flag", 1,
      std::bind(&RosOnlineDataProvider::callbackReinit, this,
                std::placeholders::_1));
  reinit_pose_subscriber_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "reinit_pose", 1,
          std::bind(&RosOnlineDataProvider::callbackReinitPose, this,
                    std::placeholders::_1));

  if (use_external_odom_) {
    external_odom_subscriber_ =
        node_->create_subscription<nav_msgs::msg::Odometry>(
            "external_odom", 1000,
            std::bind(&RosOnlineDataProvider::callbackExternalOdom, this,
                      std::placeholders::_1));
  }
}

RosOnlineDataProvider::~RosOnlineDataProvider() {
  VLOG(1) << "RosOnlineDataProvider shutting down.";
}

void RosOnlineDataProvider::waitForGroundTruthPoseIfRequested() {
  if (vio_params_.backend_params_->autoInitialize_ != 0) {
    gt_init_pose_received_ = true;
    return;
  }
  const double wait_s =
      getOrDeclare<double>(node_, "gt_pose_wait_time_s", 10.0);
  LOG(INFO) << "Waiting up to " << wait_s
            << "s for ground-truth pose on /gt_odom.";
  // Note: the parent executor is already spinning this node by the time
  // KimeraVioRos::runKimeraVio() reaches us, so the gt callback will fire
  // from there — we just sleep until the flag flips or the deadline expires.
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(wait_s);
  while (rclcpp::ok() && !gt_init_pose_received_ &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!gt_init_pose_received_) {
    LOG(WARNING) << "No ground-truth pose received; enabling autoInitialize.";
    vio_params_.backend_params_->autoInitialize_ = true;
  }
}

bool RosOnlineDataProvider::spin() {
  // In ROS 2 the executor drives subscriptions; spin() simply reports liveness
  // so that KimeraVioRos::spin can join cleanly when shutdown_ is set.
  return !shutdown_;
}

void RosOnlineDataProvider::subscribeMono() {
  mono_img_subscriber_ = std::make_unique<image_transport::Subscriber>(
      it_->subscribe("left_cam/image_raw", 1,
                     std::bind(&RosOnlineDataProvider::callbackMonoImage,
                               this, std::placeholders::_1)));
}

void RosOnlineDataProvider::subscribeStereo() {
  left_img_subscriber_.subscribe(node_, "left_cam/image_raw",
                                 std::string("raw"),
                                 rmw_qos_profile_sensor_data);
  right_img_subscriber_.subscribe(node_, "right_cam/image_raw",
                                  std::string("raw"),
                                  rmw_qos_profile_sensor_data);
  static constexpr size_t kSyncQueueSize = 10u;
  sync_img_ = std::make_unique<message_filters::Synchronizer<ImageSyncPolicy>>(
      ImageSyncPolicy(kSyncQueueSize),
      left_img_subscriber_,
      right_img_subscriber_);
  sync_img_->registerCallback(
      std::bind(&RosOnlineDataProvider::callbackStereoImages, this,
                std::placeholders::_1, std::placeholders::_2));
}

void RosOnlineDataProvider::subscribeRgbd() {
  left_img_subscriber_.subscribe(node_, "left_cam/image_raw",
                                 std::string("raw"),
                                 rmw_qos_profile_sensor_data);
  depth_img_subscriber_.subscribe(node_, "depth_cam/image_raw",
                                  std::string("raw"),
                                  rmw_qos_profile_sensor_data);
  static constexpr size_t kSyncQueueSize = 10u;
  sync_img_ = std::make_unique<message_filters::Synchronizer<ImageSyncPolicy>>(
      ImageSyncPolicy(kSyncQueueSize),
      left_img_subscriber_,
      depth_img_subscriber_);
  sync_img_->registerCallback(
      std::bind(&RosOnlineDataProvider::callbackRgbdImages, this,
                std::placeholders::_1, std::placeholders::_2));
}

void RosOnlineDataProvider::callbackMonoImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 1u);
  const CameraParams& cam_info = vio_params_.camera_params_.at(0);
  CHECK(img_msg);
  if (shutdown_) return;
  const Timestamp ts = toNs(img_msg->header.stamp);
  CHECK(left_frame_callback_)
      << "Did you forget to register the left frame callback?";
  left_frame_callback_(std::make_unique<Frame>(
      frame_count_, ts, cam_info, readRosImage(img_msg)));
  frame_count_++;
}

void RosOnlineDataProvider::callbackRgbdImages(
    const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 1u);
  const CameraParams& cam_info = vio_params_.camera_params_.at(0);
  CHECK(color_msg);
  CHECK(depth_msg);
  if (shutdown_) return;
  const Timestamp ts_color = toNs(color_msg->header.stamp);
  const Timestamp ts_depth = toNs(depth_msg->header.stamp);
  CHECK(left_frame_callback_)
      << "Did you forget to register the color frame callback?";
  left_frame_callback_(std::make_unique<Frame>(
      frame_count_, ts_color, cam_info, readRosImage(color_msg)));
  CHECK(depth_frame_callback_)
      << "Did you forget to register the depth frame callback?";
  depth_frame_callback_(std::make_unique<DepthFrame>(
      frame_count_,
      force_same_image_timestamp_ ? ts_color : ts_depth,
      readRosDepthImage(depth_msg)));
  frame_count_++;
}

void RosOnlineDataProvider::callbackStereoImages(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 2u);
  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info = vio_params_.camera_params_.at(1);

  CHECK(left_msg);
  CHECK(right_msg);
  const Timestamp ts_left = toNs(left_msg->header.stamp);
  const Timestamp ts_right = toNs(right_msg->header.stamp);

  if (shutdown_) return;

  CHECK(left_frame_callback_)
      << "Did you forget to register the left frame callback?";
  left_frame_callback_(std::make_unique<Frame>(
      frame_count_, ts_left, left_cam_info, readRosImage(left_msg)));

  CHECK(right_frame_callback_)
      << "Did you forget to register the right frame callback?";
  right_frame_callback_(std::make_unique<Frame>(
      frame_count_,
      force_same_image_timestamp_ ? ts_left : ts_right,
      right_cam_info,
      readRosImage(right_msg)));
  frame_count_++;
}

void RosOnlineDataProvider::callbackIMU(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
  VIO::ImuAccGyr imu_accgyr;
  imu_accgyr(0) = imu_msg->linear_acceleration.x;
  imu_accgyr(1) = imu_msg->linear_acceleration.y;
  imu_accgyr(2) = imu_msg->linear_acceleration.z;
  imu_accgyr(3) = imu_msg->angular_velocity.x;
  imu_accgyr(4) = imu_msg->angular_velocity.y;
  imu_accgyr(5) = imu_msg->angular_velocity.z;
  const Timestamp ts = toNs(imu_msg->header.stamp);
  if (shutdown_) return;
  CHECK(imu_single_callback_)
      << "Did you forget to register the IMU callback?";
  imu_single_callback_(ImuMeasurement(ts, imu_accgyr));
}

void RosOnlineDataProvider::callbackGtOdom(
    const nav_msgs::msg::Odometry::ConstSharedPtr& gt_odom_msg) {
  CHECK(gt_odom_msg);
  if (!gt_init_pose_received_) {
    LOG(WARNING) << "Using initial ground-truth state for initialization.";
    utils::rosOdometryToVioNavState(
        *gt_odom_msg, *node_,
        &vio_params_.backend_params_->initial_ground_truth_state_);
    gt_init_pose_received_ = true;
  }
  if (log_gt_data_) {
    logGtData(gt_odom_msg);
  } else {
    gt_odom_subscriber_.reset();
  }
}

void RosOnlineDataProvider::callbackExternalOdom(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  CHECK(odom_msg);
  VIO::VioNavState kimera_odom;
  utils::rosOdometryToVioNavState(*odom_msg, *node_, &kimera_odom);
  if (external_odom_callback_) {
    external_odom_callback_(ExternalOdomMeasurement(
        toNs(odom_msg->header.stamp),
        gtsam::NavState(kimera_odom.pose_, kimera_odom.velocity_)));
  }
}

void RosOnlineDataProvider::callbackReinit(
    const std_msgs::msg::Bool::ConstSharedPtr& msg) {
  if (msg->data) {
    reinit_flag_ = true;
    LOG(INFO) << "Reinitialization flag received.";
  }
}

void RosOnlineDataProvider::callbackReinitPose(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  gtsam::Rot3 rotation(gtsam::Quaternion(msg->pose.orientation.w,
                                         msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z));
  gtsam::Point3 position(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  reinit_packet_.setReinitPose(gtsam::Pose3(rotation, position));
}

void RosOnlineDataProvider::publishStaticTf(
    const gtsam::Pose3& pose,
    const std::string& parent_frame_id,
    const std::string& child_frame_id) {
  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.stamp = node_->get_clock()->now();
  tfs.header.frame_id = parent_frame_id;
  tfs.child_frame_id = child_frame_id;
  utils::gtsamPoseToRosTf(pose, &tfs.transform);
  static_tf_broadcaster_->sendTransform(tfs);
}

}  // namespace VIO
