/**
 * @file   RosVisualizer.cpp
 * @brief  Slice B implementation — full Visualizer3D outputs.
 */

#include "kimera_vio_ros/RosVisualizer.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>

#include <cv_bridge/cv_bridge.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/mesh/Mesh.h>

#include "kimera_vio_ros/utils/UtilsRos.hpp"

DECLARE_int32(viz_type);

namespace VIO {

namespace {

template <typename T>
T getRequired(rclcpp::Node* node, const std::string& name) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name);
  }
  T value;
  CHECK(node->get_parameter(name, value)) << "Required parameter '" << name
                                          << "' not set.";
  return value;
}

template <typename T>
T getOrDeclare(rclcpp::Node* node, const std::string& name, const T& def) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

inline builtin_interfaces::msg::Time tsToRos(VIO::Timestamp ts) {
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(ts / 1000000000LL);
  out.nanosec = static_cast<uint32_t>(ts % 1000000000LL);
  return out;
}

}  // namespace

RosVisualizer::RosVisualizer(rclcpp::Node* node, const VioParams& vio_params)
    : Visualizer3D(vio_params.frontend_type_ == FrontendType::kMonoImu
                       ? VisualizationType::kNone
                       : static_cast<VisualizationType>(FLAGS_viz_type)),
      node_(CHECK_NOTNULL(node)),
      image_size_(vio_params.camera_params_.at(0).image_size_) {
  base_link_frame_id_ = getRequired<std::string>(node_, "base_link_frame_id");
  odom_frame_id_ = getRequired<std::string>(node_, "odom_frame_id");
  map_frame_id_ = getRequired<std::string>(node_, "map_frame_id");

  // Resiliency thresholds (optional — default 0).
  pos_det_threshold_ = static_cast<float>(
      getOrDeclare<double>(node_, "position_det_threshold", 0.0));
  vel_det_threshold_ = static_cast<float>(
      getOrDeclare<double>(node_, "velocity_det_threshold", 0.0));
  mono_ransac_threshold_ =
      getOrDeclare<int>(node_, "mono_ransac_threshold", 0);
  stereo_ransac_threshold_ =
      getOrDeclare<int>(node_, "stereo_ransac_threshold", 0);

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  rclcpp::QoS basic_qos(1);

  odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "odometry", latched_qos);
  pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "time_horizon_pointcloud", latched_qos);
  mesh_3d_frame_pub_ = node_->create_publisher<pcl_msgs::msg::PolygonMesh>(
      "mesh", latched_qos);
  frontend_stats_pub_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(
          "frontend_stats", basic_qos);
  resiliency_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "resiliency", basic_qos);
  imu_bias_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "imu_bias", basic_qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

VisualizerOutput::UniquePtr RosVisualizer::spinOnce(
    const VisualizerInput& viz_input) {
  if (viz_input.frontend_output_) {
    publishFrontendOutput(viz_input.frontend_output_);
  }
  if (viz_input.backend_output_) {
    publishBackendOutput(viz_input.backend_output_);
  }
  if (viz_input.frontend_output_ && viz_input.backend_output_) {
    publishResiliency(viz_input.frontend_output_, viz_input.backend_output_);
  }
  if (viz_input.mesher_output_) {
    publishMesherOutput(viz_input.mesher_output_);
  }
  // ROS-only: visualizer never produces widgets. Display is fed via the
  // Pipeline display queue (RosDisplay).
  return std::make_unique<VisualizerOutput>();
}

void RosVisualizer::publishBackendOutput(
    const BackendOutput::ConstPtr& output) {
  CHECK(output);
  publishTf(output);
  publishState(output);
  publishImuBias(output);
  publishTimeHorizonPointCloud(output);
}

void RosVisualizer::publishFrontendOutput(
    const FrontendOutputPacketBase::ConstPtr& output) const {
  CHECK(output);
  publishFrontendStats(output);
}

void RosVisualizer::publishMesherOutput(
    const MesherOutput::ConstPtr& output) const {
  CHECK(output);
  publishPerFrameMesh3D(output);
}

void RosVisualizer::publishState(const BackendOutput::ConstPtr& output) const {
  const Timestamp& ts = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  const gtsam::Rot3& rotation = pose.rotation();
  const gtsam::Quaternion quaternion = rotation.toQuaternion();
  const gtsam::Vector3& velocity = output->W_State_Blkf_.velocity_;
  const gtsam::Matrix6 pose_cov =
      gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3 vel_cov =
      gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = tsToRos(ts);
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_link_frame_id_;

  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = pose.z();
  msg.pose.pose.orientation.w = quaternion.w();
  msg.pose.pose.orientation.x = quaternion.x();
  msg.pose.pose.orientation.y = quaternion.y();
  msg.pose.pose.orientation.z = quaternion.z();

  // GTSAM Pose3 covariance: rotation-first; ROS Odometry: translation-first.
  static const std::vector<int> remapping{3, 4, 5, 0, 1, 2};
  for (int i = 0; i < pose_cov.rows(); ++i) {
    for (int j = 0; j < pose_cov.cols(); ++j) {
      msg.pose.covariance[remapping[i] * pose_cov.cols() + remapping[j]] =
          pose_cov(i, j);
    }
  }

  const gtsam::Matrix3 inv_rot = rotation.transpose();
  const gtsam::Vector3 v_body = inv_rot * velocity;
  msg.twist.twist.linear.x = v_body(0);
  msg.twist.twist.linear.y = v_body(1);
  msg.twist.twist.linear.z = v_body(2);
  const gtsam::Matrix3 v_cov_body = inv_rot * vel_cov * rotation.matrix();
  for (int i = 0; i < v_cov_body.rows(); ++i) {
    for (int j = 0; j < v_cov_body.cols(); ++j) {
      msg.twist.covariance[i * 6 + j] = v_cov_body(i, j);
    }
  }

  odometry_pub_->publish(msg);
}

void RosVisualizer::publishTf(const BackendOutput::ConstPtr& output) {
  const Timestamp ts = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = tsToRos(ts);
  odom_tf.header.frame_id = odom_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;
  utils::gtsamPoseToRosTf(pose, &odom_tf.transform);
  tf_broadcaster_->sendTransform(odom_tf);
}

void RosVisualizer::publishTimeHorizonPointCloud(
    const BackendOutput::ConstPtr& output) const {
  const Timestamp& ts = output->timestamp_;
  const PointsWithIdMap& points_with_id = output->landmarks_with_id_map_;
  const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map =
      output->lmk_id_to_lmk_type_map_;

  if (points_with_id.empty()) return;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.is_dense = true;
  cloud.height = 1;
  cloud.width = points_with_id.size();
  cloud.points.resize(points_with_id.size());

  const bool color_the_cloud = !lmk_id_to_lmk_type_map.empty();
  if (color_the_cloud) {
    CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
  }

  size_t i = 0;
  for (const auto& id_point : points_with_id) {
    const auto& p = id_point.second;
    cloud.points[i].x = static_cast<float>(p.x());
    cloud.points[i].y = static_cast<float>(p.y());
    cloud.points[i].z = static_cast<float>(p.z());
    if (color_the_cloud) {
      const auto type_it = lmk_id_to_lmk_type_map.find(id_point.first);
      if (type_it != lmk_id_to_lmk_type_map.end()) {
        switch (type_it->second) {
          case LandmarkType::SMART:
            cloud.points[i].r = 0;   cloud.points[i].g = 255; cloud.points[i].b = 0;
            break;
          case LandmarkType::PROJECTION:
            cloud.points[i].r = 0;   cloud.points[i].g = 0;   cloud.points[i].b = 255;
            break;
          default:
            cloud.points[i].r = 255; cloud.points[i].g = 0;   cloud.points[i].b = 0;
            break;
        }
      }
    }
    ++i;
  }

  sensor_msgs::msg::PointCloud2 ros_msg;
  pcl::toROSMsg(cloud, ros_msg);
  ros_msg.header.frame_id = odom_frame_id_;
  ros_msg.header.stamp = tsToRos(ts);
  pointcloud_pub_->publish(ros_msg);
}

void RosVisualizer::publishPerFrameMesh3D(
    const MesherOutput::ConstPtr& output) const {
  const Mesh2D& mesh_2d = output->mesh_2d_;
  const Mesh3D& mesh_3d = output->mesh_3d_;
  const size_t num_polys = mesh_2d.getNumberOfPolygons();
  const size_t poly_dim = mesh_2d.getMeshPolygonDimension();
  if (num_polys == 0u) return;

  const float cam_w = static_cast<float>(image_size_.width);
  const float cam_h = static_cast<float>(image_size_.height);
  if (cam_w <= 0.0f || cam_h <= 0.0f) return;

  pcl_msgs::msg::PolygonMesh msg;
  msg.header.stamp = tsToRos(output->timestamp_);
  msg.header.frame_id = odom_frame_id_;

  pcl::PointCloud<PointNormalUV> cloud;
  cloud.points.reserve(num_polys * poly_dim);
  msg.polygons.reserve(num_polys);

  Mesh2D::Polygon polygon;
  for (size_t i = 0; i < num_polys; ++i) {
    if (!mesh_2d.getPolygon(i, &polygon)) continue;
    const LandmarkId& l0 = polygon.at(0).getLmkId();
    const LandmarkId& l1 = polygon.at(1).getLmkId();
    const LandmarkId& l2 = polygon.at(2).getLmkId();

    Mesh3D::VertexId p0_id, p1_id, p2_id;
    Mesh3D::VertexType v0, v1, v2;
    if (!(mesh_3d.getVertex(l0, &v0, &p0_id) &&
          mesh_3d.getVertex(l1, &v1, &p1_id) &&
          mesh_3d.getVertex(l2, &v2, &p2_id))) {
      continue;
    }

    const Vertex2D& px0 = polygon.at(0).getVertexPosition();
    const Vertex2D& px1 = polygon.at(1).getVertexPosition();
    const Vertex2D& px2 = polygon.at(2).getVertexPosition();
    const Vertex3D& q0 = v0.getVertexPosition();
    const Vertex3D& q1 = v1.getVertexPosition();
    const Vertex3D& q2 = v2.getVertexPosition();
    const Mesh3D::VertexNormal& n0 = v0.getVertexNormal();
    const Mesh3D::VertexNormal& n1 = v1.getVertexNormal();
    const Mesh3D::VertexNormal& n2 = v2.getVertexNormal();

    PointNormalUV pn0{}, pn1{}, pn2{};
    pn0.x = q0.x; pn1.x = q1.x; pn2.x = q2.x;
    pn0.y = q0.y; pn1.y = q1.y; pn2.y = q2.y;
    pn0.z = q0.z; pn1.z = q1.z; pn2.z = q2.z;
    pn0.u = px0.x / cam_w; pn1.u = px1.x / cam_w; pn2.u = px2.x / cam_w;
    pn0.v = px0.y / cam_h; pn1.v = px1.y / cam_h; pn2.v = px2.y / cam_h;
    pn0.normal_x = n0.x; pn1.normal_x = n1.x; pn2.normal_x = n2.x;
    pn0.normal_y = n0.y; pn1.normal_y = n1.y; pn2.normal_y = n2.y;
    pn0.normal_z = n0.z; pn1.normal_z = n1.z; pn2.normal_z = n2.z;
    cloud.points.push_back(pn0);
    cloud.points.push_back(pn1);
    cloud.points.push_back(pn2);

    pcl_msgs::msg::Vertices verts;
    verts.vertices.resize(3);
    const size_t idx = i * poly_dim;
    // Reverse winding for RViz.
    verts.vertices[0] = idx + 2;
    verts.vertices[1] = idx + 1;
    verts.vertices[2] = idx;
    msg.polygons.push_back(verts);
  }

  cloud.is_dense = false;
  cloud.width = cloud.points.size();
  cloud.height = 1;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  msg.cloud = cloud_msg;
  msg.cloud.header = msg.header;

  if (!msg.polygons.empty()) {
    mesh_3d_frame_pub_->publish(msg);
  }
}

void RosVisualizer::publishFrontendStats(
    const FrontendOutputPacketBase::ConstPtr& output) const {
  const DebugTrackerInfo& info = output->getTrackerInfo();
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(13);
  msg.data[0] = info.nrDetectedFeatures_;
  msg.data[1] = info.nrTrackerFeatures_;
  msg.data[2] = info.nrMonoInliers_;
  msg.data[3] = info.nrMonoPutatives_;
  msg.data[4] = info.nrStereoInliers_;
  msg.data[5] = info.nrStereoPutatives_;
  msg.data[6] = info.monoRansacIters_;
  msg.data[7] = info.stereoRansacIters_;
  msg.data[8] = info.nrValidRKP_;
  msg.data[9] = info.nrNoLeftRectRKP_;
  msg.data[10] = info.nrNoRightRectRKP_;
  msg.data[11] = info.nrNoDepthRKP_;
  msg.data[12] = info.nrFailedArunRKP_;
  std_msgs::msg::MultiArrayDimension dim;
  dim.size = msg.data.size();
  dim.stride = 1;
  dim.label =
      "nrDetFeat,nrTrackFeat,nrMoIn,nrMoPu,nrStIn,nrStPu,"
      "moRaIt,stRaIt,nrVaRKP,nrNoLRKP,nrNoRRKP,nrNoDRKP,nrFaARKP";
  msg.layout.dim.push_back(dim);
  frontend_stats_pub_->publish(msg);
}

void RosVisualizer::publishResiliency(
    const FrontendOutputPacketBase::ConstPtr& frontend_output,
    const BackendOutput::ConstPtr& backend_output) const {
  const DebugTrackerInfo& info = frontend_output->getTrackerInfo();
  const gtsam::Matrix6 pose_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3 vel_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 6, 9, 6, 9);
  CHECK_EQ(pose_cov.size(), 36);
  gtsam::Matrix3 position_cov = gtsam::sub(pose_cov, 3, 6, 3, 6);
  CHECK_EQ(position_cov.size(), 9);

  gtsam::Matrix U, V;
  gtsam::Vector cov_v_eigv, cov_p_eigv;
  gtsam::svd(vel_cov, U, cov_v_eigv, V);
  gtsam::svd(position_cov, U, cov_p_eigv, V);

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(8);
  msg.data[0] = std::cbrt(cov_p_eigv(0) * cov_p_eigv(1) * cov_p_eigv(2));
  msg.data[1] = std::cbrt(cov_v_eigv(0) * cov_v_eigv(1) * cov_v_eigv(2));
  msg.data[2] = info.nrStereoInliers_;
  msg.data[3] = info.nrMonoInliers_;
  msg.data[4] = pos_det_threshold_;
  msg.data[5] = vel_det_threshold_;
  msg.data[6] = stereo_ransac_threshold_;
  msg.data[7] = mono_ransac_threshold_;

  std_msgs::msg::MultiArrayDimension dim;
  dim.size = msg.data.size();
  dim.stride = 1;
  dim.label =
      "Values: cbrtPDet, cbrtVDet, nrStIn, nrMoIn. "
      "Thresholds: cbrtPDet, cbrtVDet, nrStIn, nrMoIn.";
  msg.layout.dim.push_back(dim);
  resiliency_pub_->publish(msg);
}

void RosVisualizer::publishImuBias(const BackendOutput::ConstPtr& output) const {
  const ImuBias& imu_bias = output->W_State_Blkf_.imu_bias_;
  const Vector3& accel_bias = imu_bias.accelerometer();
  const Vector3& gyro_bias = imu_bias.gyroscope();
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(6);
  msg.data[0] = gyro_bias[0];
  msg.data[1] = gyro_bias[1];
  msg.data[2] = gyro_bias[2];
  msg.data[3] = accel_bias[0];
  msg.data[4] = accel_bias[1];
  msg.data[5] = accel_bias[2];
  std_msgs::msg::MultiArrayDimension dim;
  dim.size = msg.data.size();
  dim.stride = 1;
  dim.label = "Gyro Bias: x,y,z. Accel Bias: x,y,z";
  msg.layout.dim.push_back(dim);
  imu_bias_pub_->publish(msg);
}

void RosVisualizer::publishDebugImage(const Timestamp& timestamp,
                                      const cv::Mat& debug_image) const {
  static constexpr const char* kName = "mesh_2d";
  auto it = image_pubs_.find(kName);
  if (it == image_pubs_.end()) {
    rclcpp::QoS qos(1);
    qos.transient_local();
    auto pub = image_transport::create_publisher(
        node_, std::string(kName) + "/image_raw", qos.get_rmw_qos_profile());
    auto inserted = image_pubs_.emplace(kName, std::move(pub));
    it = inserted.first;
  }
  std_msgs::msg::Header h;
  h.stamp = tsToRos(timestamp);
  h.frame_id = base_link_frame_id_;
  it->second.publish(
      cv_bridge::CvImage(h, "bgr8", debug_image).toImageMsg());
}

}  // namespace VIO
