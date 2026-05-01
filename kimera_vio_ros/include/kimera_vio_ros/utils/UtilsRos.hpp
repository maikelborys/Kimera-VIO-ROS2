/**
 * @file   UtilsRos.hpp
 * @brief  ROS 2 conversion helpers (gtsam <-> ROS).
 *
 * Port of Kimera-VIO-ROS/include/kimera_vio_ros/utils/UtilsRos.h.
 */

#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Pose3.h>

#include <kimera-vio/common/VioNavState.h>

namespace VIO {
namespace utils {

void rosTfToGtsamPose(const geometry_msgs::msg::Transform& tf,
                      gtsam::Pose3* pose);

void gtsamPoseToRosTf(const gtsam::Pose3& pose,
                      geometry_msgs::msg::Transform* tf);

void rosOdometryToGtsamPose(const nav_msgs::msg::Odometry& odom,
                            gtsam::Pose3* pose);

void rosOdometryToVioNavState(const nav_msgs::msg::Odometry& gt_odom,
                              const rclcpp::Node& node,
                              VioNavState* vio_navstate);

}  // namespace utils
}  // namespace VIO
