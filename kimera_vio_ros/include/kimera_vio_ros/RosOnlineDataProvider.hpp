/**
 * @file   RosOnlineDataProvider.hpp
 * @brief  Live (rclcpp) data provider for Kimera-VIO. Slice A: stereo + IMU.
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <kimera-vio/frontend/StereoImuSyncPacket.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>

#include "kimera_vio_ros/RosDataProviderInterface.hpp"

namespace VIO {

class RosOnlineDataProvider : public RosDataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosOnlineDataProvider);
  KIMERA_POINTER_TYPEDEFS(RosOnlineDataProvider);

  RosOnlineDataProvider(rclcpp::Node* node, const VioParams& vio_params);
  ~RosOnlineDataProvider() override;

  /**
   * Parallel mode only: the Pipeline runs in its own threads (driven by
   * KimeraVioRos::spin()); subscriptions are serviced by the rclcpp executor
   * on the parent node. spin() reports liveness so KimeraVioRos can join.
   */
  bool spin() override;

  bool getReinitFlag() const { return reinit_flag_; }
  void resetReinitFlag() { reinit_packet_.resetReinitFlag(); }

 private:
  // Subscription helpers (called from ctor based on frontend type).
  void subscribeMono();
  void subscribeStereo();
  void subscribeRgbd();

  // Subscriber callbacks.
  void callbackMonoImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);
  void callbackStereoImages(
      const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);
  void callbackRgbdImages(
      const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
  void callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
  void callbackGtOdom(
      const nav_msgs::msg::Odometry::ConstSharedPtr& gt_odom_msg);
  void callbackExternalOdom(
      const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);
  void callbackReinit(const std_msgs::msg::Bool::ConstSharedPtr& msg);
  void callbackReinitPose(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

  void publishStaticTf(const gtsam::Pose3& pose,
                       const std::string& parent_frame_id,
                       const std::string& child_frame_id);

  void waitForGroundTruthPoseIfRequested();

 private:
  using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::SubscriberFilter left_img_subscriber_;
  image_transport::SubscriberFilter right_img_subscriber_;
  image_transport::SubscriberFilter depth_img_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<ImageSyncPolicy>> sync_img_;
  // Mono path uses a plain image_transport subscriber (no sync needed).
  std::unique_ptr<image_transport::Subscriber> mono_img_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      external_odom_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reinit_flag_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      reinit_pose_subscriber_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Dedicated callback group for the IMU subscription (reentrant: parallel
  // delivery; the IMU callback is fast and pushes into a thread-safe queue).
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;

  // State.
  FrameId frame_count_;
  bool reinit_flag_;
  ReinitPacket reinit_packet_;
  bool force_same_image_timestamp_;
  std::atomic_bool gt_init_pose_received_{false};
  bool use_external_odom_;

  std::string base_link_frame_id_;
  std::string left_cam_frame_id_;
  std::string right_cam_frame_id_;
};

}  // namespace VIO
