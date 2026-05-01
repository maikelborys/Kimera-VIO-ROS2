/**
 * @file   KimeraVioRos.hpp
 * @brief  ROS 2 wrapper around the Kimera-VIO Pipeline.
 *
 * Slice A scope: stereo+IMU online provider, minimal RosVisualizer,
 * no LCD, no display, no rosbag (use `ros2 bag play` instead).
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>

#include "kimera_vio_ros/LcdRegistrationServer.hpp"
#include "kimera_vio_ros/RosDataProviderInterface.hpp"
#include "kimera_vio_ros/RosDisplay.hpp"
#include "kimera_vio_ros/RosLoopClosureVisualizer.hpp"
#include "kimera_vio_ros/RosVisualizer.hpp"

namespace VIO {

/**
 * Owns the rclcpp::Node, the Pipeline, the data provider and the visualizer.
 * The node itself hosts every subscription/publisher created by data
 * providers and visualizer (single TF tree, single parameter namespace).
 */
class KimeraVioRos {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(KimeraVioRos);
  KIMERA_POINTER_TYPEDEFS(KimeraVioRos);

  explicit KimeraVioRos(const rclcpp::NodeOptions& opts);
  virtual ~KimeraVioRos();

  /** Build the pipeline and return the node so an executor can spin it. */
  rclcpp::Node::SharedPtr node() const { return node_; }

  /** Start pipeline + data provider threads. Returns when pipeline finishes. */
  bool runKimeraVio();

  /** Cooperative shutdown (joins worker threads). */
  void shutdown();

 protected:
  bool spin();
  RosDataProviderInterface::UniquePtr createDataProvider(
      const VioParams& vio_params);
  void connectVIO();
  void restartCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

 protected:
  rclcpp::Node::SharedPtr node_;

  VioParams::Ptr vio_params_;
  Pipeline::UniquePtr vio_pipeline_;

  bool use_rviz_;
  bool use_lcd_registration_server_;
  RosDataProviderInterface::UniquePtr data_provider_;
  RosVisualizer::UniquePtr ros_visualizer_;
  RosDisplay::UniquePtr ros_display_;
  RosLoopClosureVisualizer::UniquePtr ros_lcd_visualizer_;
  std::unique_ptr<LcdRegistrationServer> lcd_registration_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_vio_pipeline_srv_;
  std::atomic_bool restart_vio_pipeline_;
  std::atomic_bool shutting_down_;
};

}  // namespace VIO
