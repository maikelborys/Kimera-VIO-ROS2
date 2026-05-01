/**
 * @file   LcdRegistrationServer.hpp
 * @brief  Slice C: ROS 2 port of the LcdRegistrationServer.
 *
 * Advertises ~/register_lcd_frames service backed by the live LcdModule of
 * a running pipeline. Service runs on a reentrant CallbackGroup so the
 * MultiThreadedExecutor can dispatch it without blocking other callbacks
 * (Pipeline.registerFrames may take tens of ms under load).
 */

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <pose_graph_tools_msgs/srv/lcd_frame_registration.hpp>

#include <kimera-vio/loopclosure/LcdModule.h>

namespace VIO {

class LcdRegistrationServer {
 public:
  using LcdFrameRegistrationSrv =
      pose_graph_tools_msgs::srv::LcdFrameRegistration;

  LcdRegistrationServer(rclcpp::Node* node, LcdModule* lcd_module);
  ~LcdRegistrationServer() = default;

  /// Tear down the service handle. Safe to call from any thread.
  void stop();

 private:
  void onRequest(
      const std::shared_ptr<LcdFrameRegistrationSrv::Request> req,
      std::shared_ptr<LcdFrameRegistrationSrv::Response> res);

  rclcpp::Node* node_;
  LcdModule* lcd_module_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<LcdFrameRegistrationSrv>::SharedPtr server_;
};

}  // namespace VIO
