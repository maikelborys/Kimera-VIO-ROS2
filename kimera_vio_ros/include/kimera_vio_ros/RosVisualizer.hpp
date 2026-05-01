/**
 * @file   RosVisualizer.hpp
 * @brief  Slice B: full Visualizer3D - publishes odometry/tf/mesh/pointcloud/
 *         frontend_stats/resiliency/imu_bias and debug images.
 */

#pragma once

#define PCL_NO_PRECOMPILE  // before any PCL header so our custom point type
                           //  registers with the templated algorithms

#include <memory>
#include <string>
#include <unordered_map>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_types.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core.hpp>

#include <kimera-vio/backend/VioBackend-definitions.h>
#include <kimera-vio/frontend/VisionImuFrontend-definitions.h>
#include <kimera-vio/mesh/Mesher-definitions.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

namespace VIO {

/**
 * Custom PCL point type carrying texture coords + normals, used in the
 * 3D-mesh-with-2D-image-uv published on /kimera_vio_ros/mesh.
 */
struct PointNormalUV {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float u;
  float v;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class RosVisualizer : public Visualizer3D {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosVisualizer);
  KIMERA_POINTER_TYPEDEFS(RosVisualizer);

  RosVisualizer(rclcpp::Node* node, const VioParams& vio_params);
  ~RosVisualizer() override = default;

  VisualizerOutput::UniquePtr spinOnce(const VisualizerInput& input) override;

 private:
  void publishBackendOutput(const BackendOutput::ConstPtr& output);
  void publishFrontendOutput(
      const FrontendOutputPacketBase::ConstPtr& output) const;
  void publishMesherOutput(const MesherOutput::ConstPtr& output) const;

  void publishState(const BackendOutput::ConstPtr& output) const;
  void publishTf(const BackendOutput::ConstPtr& output);
  void publishTimeHorizonPointCloud(const BackendOutput::ConstPtr& output) const;
  void publishPerFrameMesh3D(const MesherOutput::ConstPtr& output) const;
  void publishFrontendStats(
      const FrontendOutputPacketBase::ConstPtr& output) const;
  void publishResiliency(
      const FrontendOutputPacketBase::ConstPtr& frontend_output,
      const BackendOutput::ConstPtr& backend_output) const;
  void publishImuBias(const BackendOutput::ConstPtr& output) const;
  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;

  rclcpp::Node* node_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_3d_frame_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      frontend_stats_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr resiliency_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_bias_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Lazy-cached image_transport publishers, keyed by name (e.g. "mesh_2d").
  mutable std::unordered_map<std::string, image_transport::Publisher>
      image_pubs_;

  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;
  cv::Size image_size_;

  // Cached resiliency thresholds (so we don't fetch every spin).
  float pos_det_threshold_{0.0f};
  float vel_det_threshold_{0.0f};
  int mono_ransac_threshold_{0};
  int stereo_ransac_threshold_{0};
};

}  // namespace VIO

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VIO::PointNormalUV,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float,
        normal_y,
        normal_y)(float, normal_z, normal_z)(float, u, u)(float, v, v))
