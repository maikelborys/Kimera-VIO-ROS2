/**
 * @file   UtilsRos.cpp
 * @brief  ROS 2 conversion helpers (gtsam <-> ROS).
 */

#include "kimera_vio_ros/utils/UtilsRos.hpp"

#include <vector>

#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

namespace VIO {
namespace utils {

void rosTfToGtsamPose(const geometry_msgs::msg::Transform& tf,
                      gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);
  *pose = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
          tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)),
      gtsam::Point3(tf.translation.x, tf.translation.y, tf.translation.z));
}

void gtsamPoseToRosTf(const gtsam::Pose3& pose,
                      geometry_msgs::msg::Transform* tf) {
  CHECK_NOTNULL(tf);
  tf->translation.x = pose.x();
  tf->translation.y = pose.y();
  tf->translation.z = pose.z();
  const gtsam::Quaternion& quat = pose.rotation().toQuaternion();
  tf->rotation.w = quat.w();
  tf->rotation.x = quat.x();
  tf->rotation.y = quat.y();
  tf->rotation.z = quat.z();
}

void rosOdometryToGtsamPose(const nav_msgs::msg::Odometry& odom,
                            gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(odom.pose.pose.orientation.w,
                                                 odom.pose.pose.orientation.x,
                                                 odom.pose.pose.orientation.y,
                                                 odom.pose.pose.orientation.z);
  gtsam::Point3 translation(odom.pose.pose.position.x,
                            odom.pose.pose.position.y,
                            odom.pose.pose.position.z);
  *pose = gtsam::Pose3(rotation, translation);
}

void rosOdometryToVioNavState(const nav_msgs::msg::Odometry& odom,
                              const rclcpp::Node& node,
                              VioNavState* vio_navstate) {
  CHECK_NOTNULL(vio_navstate);
  rosOdometryToGtsamPose(odom, &vio_navstate->pose_);

  gtsam::Vector3 body_world_Vel_body(odom.twist.twist.linear.x,
                                     odom.twist.twist.linear.y,
                                     odom.twist.twist.linear.z);
  const gtsam::Rot3 world_R_body = vio_navstate->pose_.rotation();
  vio_navstate->velocity_ = world_R_body * body_world_Vel_body;

  // Re-declare on a const_cast: declare_parameter requires non-const, but in
  // ROS 2 callers pass the node from a const context. Use get_parameter_or
  // for safety against undeclared params.
  std::vector<double> parsed_acc_bias = {0.0, 0.0, 0.0};
  std::vector<double> parsed_gyr_bias = {0.0, 0.0, 0.0};
  node.get_parameter_or<std::vector<double>>("gt_accel_bias", parsed_acc_bias,
                                             {0.0, 0.0, 0.0});
  node.get_parameter_or<std::vector<double>>("gt_gyro_bias", parsed_gyr_bias,
                                             {0.0, 0.0, 0.0});
  CHECK_EQ(parsed_acc_bias.size(), 3u);
  CHECK_EQ(parsed_gyr_bias.size(), 3u);

  gtsam::Vector3 acc_bias(parsed_acc_bias[0], parsed_acc_bias[1],
                          parsed_acc_bias[2]);
  gtsam::Vector3 gyr_bias(parsed_gyr_bias[0], parsed_gyr_bias[1],
                          parsed_gyr_bias[2]);
  vio_navstate->imu_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyr_bias);
}

}  // namespace utils
}  // namespace VIO
