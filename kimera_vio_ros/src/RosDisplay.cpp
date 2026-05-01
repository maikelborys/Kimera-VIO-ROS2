/**
 * @file   RosDisplay.cpp
 */

#include "kimera_vio_ros/RosDisplay.hpp"

#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <glog/logging.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/header.hpp>

#include <kimera-vio/visualizer/Display-definitions.h>
#include <kimera-vio/visualizer/Visualizer3D-definitions.h>

namespace VIO {

namespace {

template <typename T>
T getOrDeclare(rclcpp::Node* node, const std::string& name, const T& def) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

}  // namespace

RosDisplay::RosDisplay(rclcpp::Node* node)
    : DisplayBase(VIO::DisplayType::kOpenCV),
      node_(CHECK_NOTNULL(node)) {
  base_link_frame_id_ =
      getOrDeclare<std::string>(node_, "base_link_frame_id", "base_link");
}

void RosDisplay::spinOnce(DisplayInputBase::UniquePtr&& viz_output) {
  CHECK(viz_output);
  publishImages(*viz_output);
}

void RosDisplay::publishImages(const DisplayInputBase& viz_output) {
  std_msgs::msg::Header header;
  header.stamp.sec =
      static_cast<int32_t>(viz_output.timestamp_ / 1000000000LL);
  header.stamp.nanosec =
      static_cast<uint32_t>(viz_output.timestamp_ % 1000000000LL);
  header.frame_id = base_link_frame_id_;

  for (const ImageToDisplay& img_to_display : viz_output.images_to_display_) {
    const std::string& name = img_to_display.name_;
    auto it = image_pubs_.find(name);
    if (it == image_pubs_.end()) {
      rclcpp::QoS qos(1);
      auto pub = image_transport::create_publisher(
          node_, name + "/image_raw", qos.get_rmw_qos_profile());
      auto inserted = image_pubs_.emplace(name, std::move(pub));
      it = inserted.first;
    }
    it->second.publish(
        cv_bridge::CvImage(header, "bgr8", img_to_display.image_).toImageMsg());
  }
}

}  // namespace VIO
