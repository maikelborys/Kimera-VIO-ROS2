/**
 * @file   RosDisplay.hpp
 * @brief  ROS 2 implementation of DisplayBase. Publishes Kimera's debug 2D
 *         images (e.g. tracker overlay, mesh-2D) over image_transport.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <image_transport/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <kimera-vio/visualizer/Display.h>

namespace VIO {

class RosDisplay : public DisplayBase {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDisplay);
  KIMERA_POINTER_TYPEDEFS(RosDisplay);

  explicit RosDisplay(rclcpp::Node* node);
  ~RosDisplay() override = default;

  void spinOnce(DisplayInputBase::UniquePtr&& viz_output) override;

 private:
  void publishImages(const DisplayInputBase& viz_output);

  rclcpp::Node* node_;
  std::string base_link_frame_id_;

  // Lazy-cached image_transport publishers, keyed by display name.
  std::unordered_map<std::string, image_transport::Publisher> image_pubs_;
};

}  // namespace VIO
