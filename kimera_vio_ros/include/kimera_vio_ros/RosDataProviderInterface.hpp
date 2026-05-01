/**
 * @file   RosDataProviderInterface.hpp
 * @brief  Base ROS 2 data provider for the Kimera-VIO pipeline.
 */

#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/logging/Logger.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>

namespace VIO {

/**
 * Holds shared state for any ROS 2 data provider:
 *  - the parent rclcpp::Node (used for parameters, subs, clock)
 *  - the VioParams copy
 *  - utility readers for ROS 2 sensor_msgs::Image -> cv::Mat
 *  - optional ground-truth CSV logger
 */
class RosDataProviderInterface : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(RosDataProviderInterface);

  RosDataProviderInterface(rclcpp::Node* node, const VioParams& vio_params);
  ~RosDataProviderInterface() override;

  /** Thread-safe accessor over the inherited atomic shutdown flag. */
  bool isShutdown() const { return shutdown_.load(); }

 protected:
  cv::Mat readRosImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  cv::Mat readRosDepthImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  void logGtData(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry);

  void printParsedParams() const;

 protected:
  // Non-owning. Lifetime managed by the executor / main().
  rclcpp::Node* node_;
  VioParams vio_params_;

  bool log_gt_data_;
  bool is_header_written_poses_vio_;
  OfstreamWrapper::Ptr output_gt_poses_csv_;
};

}  // namespace VIO
