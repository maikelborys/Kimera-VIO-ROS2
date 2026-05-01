/**
 * @file   LcdRegistrationServer.cpp
 * @brief  Slice C: ROS 2 port. See header.
 */

#include "kimera_vio_ros/LcdRegistrationServer.hpp"

#include <Eigen/Geometry>
#include <glog/logging.h>

namespace VIO {

LcdRegistrationServer::LcdRegistrationServer(rclcpp::Node* node,
                                              LcdModule* lcd_module)
    : node_(CHECK_NOTNULL(node)), lcd_module_(CHECK_NOTNULL(lcd_module)) {
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  server_ = node_->create_service<LcdFrameRegistrationSrv>(
      "~/register_lcd_frames",
      std::bind(&LcdRegistrationServer::onRequest, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);
}

void LcdRegistrationServer::stop() { server_.reset(); }

void LcdRegistrationServer::onRequest(
    const std::shared_ptr<LcdFrameRegistrationSrv::Request> req,
    std::shared_ptr<LcdFrameRegistrationSrv::Response> res) {
  CHECK(req);
  CHECK(res);
  VLOG(1) << "[LCD] registering " << req->query << " -> " << req->match;
  const auto result = lcd_module_->registerFrames(req->query, req->match);
  res->valid = result.isLoop();

  // Field rename ROS 1 -> ROS 2: match_T_query -> match_from_query (see
  // SLICE_C_MSG_COMPAT.md). Semantics unchanged: pose of query frame in match.
  const Eigen::Vector3d t = result.relative_pose_.translation();
  res->match_from_query.position.x = t.x();
  res->match_from_query.position.y = t.y();
  res->match_from_query.position.z = t.z();
  const Eigen::Quaterniond q(result.relative_pose_.rotation().matrix());
  res->match_from_query.orientation.x = q.x();
  res->match_from_query.orientation.y = q.y();
  res->match_from_query.orientation.z = q.z();
  res->match_from_query.orientation.w = q.w();
}

}  // namespace VIO
