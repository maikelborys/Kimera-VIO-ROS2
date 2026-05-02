/**
 * @file   RosLoopClosureVisualizer.cpp
 * @brief  Slice C: ROS 2 port. See header for topic surface.
 */

#include "kimera_vio_ros/RosLoopClosureVisualizer.hpp"

#include <chrono>
#include <utility>

#include <glog/logging.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cv_bridge/cv_bridge.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>

#include "kimera_vio_ros/utils/UtilsRos.hpp"

namespace VIO {

namespace {

// A previously-declared parameter (e.g. via NodeOptions
// `automatically_declare_parameters_from_overrides`) cannot be re-declared.
// Wrapping it makes the helper safe to call from multiple constructors that
// share param names with RosVisualizer.
template <typename T>
T getOrDeclare(rclcpp::Node* node, const std::string& name, const T& def) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

// Extract a 6x6 covariance from a gtsam factor's noise model when possible.
// Hydra-RPGO weights edges by this; ROS 1 wrapper left zeros (TODO in source).
// Order of gtsam Pose3 tangent space is [rx ry rz tx ty tz] — same convention
// the ROS msg consumers (Hydra) already expect for between-factor edges.
// Both Backend's ODOM BetweenFactor<Pose3> and LCD's LOOPCLOSE BetweenFactor<Pose3>
// use gtsam::noiseModel::Diagonal (subclass of Gaussian), so a direct cast
// succeeds for both. Empirically verified on EuRoC MH_03_medium: typeid is
// gtsam::noiseModel::Diagonal in every observed call.
void fillEdgeCovariance(const gtsam::SharedNoiseModel& noise_model,
                        std::array<double, 36>* out) {
  for (auto& v : *out) v = 0.0;
  if (!noise_model) return;
  const auto gauss =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise_model);
  if (!gauss) return;
  const gtsam::Matrix cov = gauss->covariance();
  if (cov.rows() != 6 || cov.cols() != 6) return;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      (*out)[6 * r + c] = cov(r, c);
    }
  }
}

}  // namespace

using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;

RosLoopClosureVisualizer::RosLoopClosureVisualizer(rclcpp::Node* node)
    : node_(CHECK_NOTNULL(node)),
      robot_id_(0),
      bow_batch_size_(5),
      bow_skip_num_(1),
      publish_vlc_frames_(true) {
  // Frame IDs are required and shared with RosVisualizer; getOrDeclare
  // tolerates pre-declaration through NodeOptions overrides.
  odom_frame_id_ =
      getOrDeclare<std::string>(node_, "odom_frame_id", std::string("odom"));
  base_link_frame_id_ = getOrDeclare<std::string>(node_, "base_link_frame_id",
                                                   std::string("base_link"));
  map_frame_id_ =
      getOrDeclare<std::string>(node_, "map_frame_id", std::string("map"));
  CHECK(!odom_frame_id_.empty());
  CHECK(!base_link_frame_id_.empty());
  CHECK(!map_frame_id_.empty());

  const int robot_id_in = getOrDeclare<int>(node_, "robot_id", 0);
  CHECK_GE(robot_id_in, 0);
  robot_id_ = static_cast<uint16_t>(robot_id_in);
  bow_batch_size_ = getOrDeclare<int>(node_, "bow_batch_size", 5);
  bow_skip_num_ = getOrDeclare<int>(node_, "bow_skip_num", 1);
  publish_vlc_frames_ = getOrDeclare<bool>(node_, "publish_vlc_frames", true);

  RCLCPP_INFO(node_->get_logger(), "BoW vector batch size: %d",
              bow_batch_size_);
  RCLCPP_INFO(node_->get_logger(), "BoW vector skip num: %d", bow_skip_num_);
  RCLCPP_INFO(node_->get_logger(), "Publish VLC frames: %d",
              publish_vlc_frames_);

  // Reentrant group so the timer + service can dispatch in parallel with
  // RosVisualizer/data-provider callbacks under the MultiThreadedExecutor.
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  trajectory_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "optimized_trajectory", rclcpp::QoS(1).reliable());
  posegraph_pub_ = node_->create_publisher<PoseGraphMsg>(
      "pose_graph", rclcpp::QoS(1).reliable());
  // Hydra consumes pose_graph_incremental and may lag the producer by several
  // keyframes; deep queue keeps backpressure off.
  posegraph_incremental_pub_ = node_->create_publisher<PoseGraphMsg>(
      "pose_graph_incremental", rclcpp::QoS(1000).reliable());
  odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "optimized_odometry", rclcpp::QoS(1).reliable());
  bow_query_pub_ = node_->create_publisher<BowQueriesMsg>(
      "bow_query", rclcpp::QoS(1000).reliable());
  vlc_frame_pub_ = node_->create_publisher<VLCFramesMsg>(
      "vlc_frames", rclcpp::QoS(100).reliable());

  vlc_frame_server_ = node_->create_service<VLCFrameQuerySrv>(
      "~/vlc_frame_query",
      std::bind(&RosLoopClosureVisualizer::onVLCFrameQuery, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      callback_group_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  for (uint16_t robot_id = 0; robot_id <= robot_id_; ++robot_id) {
    BowQueriesMsg msg;
    msg.destination_robot_id = robot_id;
    bow_queries_[robot_id] = msg;
  }
  new_frames_msg_.destination_robot_id = robot_id_;

  publish_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RosLoopClosureVisualizer::publishTimerCallback, this),
      callback_group_);
}

void RosLoopClosureVisualizer::publishLcdOutput(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);
  frames_.push_back(lcd_frame(*lcd_output));

  processBowQuery();
  if (publish_vlc_frames_) {
    const size_t pose_id = frames_.size() - 1;
    VLCFrameMsg frame_msg;
    if (getFrameMsg(static_cast<int>(pose_id), frame_msg)) {
      new_frames_msg_.frames.push_back(frame_msg);
    }
  }

  publishTf(lcd_output);
  if (trajectory_pub_->get_subscription_count() > 0) {
    publishOptimizedTrajectory(lcd_output);
  }
  if (posegraph_pub_->get_subscription_count() > 0 ||
      posegraph_incremental_pub_->get_subscription_count() > 0) {
    publishPoseGraph(lcd_output);
  }
}

void RosLoopClosureVisualizer::publishOptimizedTrajectory(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  const Timestamp& ts = lcd_output->timestamp_;
  const FrameIDTimestampMap& times = lcd_output->timestamp_map_;
  const gtsam::Values& trajectory = lcd_output->states_;

  nav_msgs::msg::Path path;
  path.poses.reserve(trajectory.size());
  for (size_t i = 0; i < trajectory.size(); i++) {
    const gtsam::Pose3 pose = trajectory.at<gtsam::Pose3>(i);
    const gtsam::Point3 trans = pose.translation();
    const gtsam::Quaternion quat = pose.rotation().toQuaternion();

    geometry_msgs::msg::PoseStamped ps_msg;
    CHECK(times.count(i));
    ps_msg.header.stamp = rclcpp::Time(times.at(i));
    ps_msg.header.frame_id = map_frame_id_;
    ps_msg.pose.position.x = trans.x();
    ps_msg.pose.position.y = trans.y();
    ps_msg.pose.position.z = trans.z();
    ps_msg.pose.orientation.x = quat.x();
    ps_msg.pose.orientation.y = quat.y();
    ps_msg.pose.orientation.z = quat.z();
    ps_msg.pose.orientation.w = quat.w();
    path.poses.push_back(ps_msg);
  }
  path.header.stamp = rclcpp::Time(ts);
  path.header.frame_id = map_frame_id_;
  trajectory_pub_->publish(path);

  // Note (carried from ROS 1): does not update odom <-> base_link transform;
  // base_link still rides on unoptimized VIO odometry.
  const gtsam::Pose3 latest_pose =
      trajectory.at<gtsam::Pose3>(trajectory.size() - 1);
  const gtsam::Point3 trans = latest_pose.translation();
  const gtsam::Quaternion quat = latest_pose.rotation().toQuaternion();
  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.header.stamp = rclcpp::Time(ts);
  odometry_msg.header.frame_id = map_frame_id_;
  odometry_msg.child_frame_id = base_link_frame_id_;
  odometry_msg.pose.pose.position.x = trans.x();
  odometry_msg.pose.pose.position.y = trans.y();
  odometry_msg.pose.pose.position.z = trans.z();
  odometry_msg.pose.pose.orientation.w = quat.w();
  odometry_msg.pose.pose.orientation.x = quat.x();
  odometry_msg.pose.pose.orientation.y = quat.y();
  odometry_msg.pose.pose.orientation.z = quat.z();
  odometry_pub_->publish(odometry_msg);
}

void RosLoopClosureVisualizer::updateRejectedEdges() {
  for (auto& loop_closure_edge : loop_closure_edges_) {
    bool is_inlier = false;
    for (const auto& inlier_edge : inlier_edges_) {
      if (loop_closure_edge.key_from == inlier_edge.key_from &&
          loop_closure_edge.key_to == inlier_edge.key_to) {
        is_inlier = true;
        continue;
      }
    }
    if (!is_inlier) {
      loop_closure_edge.type = PoseGraphEdgeMsg::REJECTED_LOOPCLOSE;
    }
  }
  for (const auto& inlier_edge : inlier_edges_) {
    bool previously_stored = false;
    for (const auto& loop_closure_edge : loop_closure_edges_) {
      if (inlier_edge.key_from == loop_closure_edge.key_from &&
          inlier_edge.key_to == loop_closure_edge.key_to) {
        previously_stored = true;
        continue;
      }
    }
    if (!previously_stored) {
      loop_closure_edges_.push_back(inlier_edge);
    }
  }
}

void RosLoopClosureVisualizer::updateNodesAndEdges(
    const FrameIDTimestampMap& times,
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  inlier_edges_.clear();
  odometry_edges_.clear();
  for (size_t i = 0; i < nfg.size(); i++) {
    const auto factor = dynamic_cast<const PoseBetween*>(nfg[i].get());
    if (!factor) continue;

    PoseGraphEdgeMsg edge;
    edge.header.frame_id = map_frame_id_;
    edge.key_from = factor->front();
    edge.key_to = factor->back();
    edge.robot_from = robot_id_;
    edge.robot_to = robot_id_;
    edge.type = (edge.key_to == edge.key_from + 1)
                    ? PoseGraphEdgeMsg::ODOM
                    : PoseGraphEdgeMsg::LOOPCLOSE;

    const gtsam::Point3& translation = factor->measured().translation();
    edge.pose.position.x = translation.x();
    edge.pose.position.y = translation.y();
    edge.pose.position.z = translation.z();
    const gtsam::Quaternion& quaternion =
        factor->measured().rotation().toQuaternion();
    edge.pose.orientation.x = quaternion.x();
    edge.pose.orientation.y = quaternion.y();
    edge.pose.orientation.z = quaternion.z();
    edge.pose.orientation.w = quaternion.w();

    // Hydra-RPGO needs non-zero covariance to weight factors; the ROS 1
    // reference left these zero (TODO in source).
    fillEdgeCovariance(factor->noiseModel(), &edge.covariance);

    if (edge.type == PoseGraphEdgeMsg::ODOM) {
      odometry_edges_.push_back(edge);
    } else {
      inlier_edges_.push_back(edge);
    }
  }

  updateRejectedEdges();

  pose_graph_nodes_.clear();
  const gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    PoseGraphNodeMsg node;
    node.key = key_list[i];
    node.robot_id = robot_id_;

    const uint64_t frame_id = gtsam::Symbol(node.key).index();
    CHECK(times.count(frame_id));
    node.header.stamp = rclcpp::Time(times.at(frame_id));

    const gtsam::Pose3& value = values.at<gtsam::Pose3>(i);
    const gtsam::Point3& translation = value.translation();
    const gtsam::Quaternion& quaternion = value.rotation().toQuaternion();
    node.pose.position.x = translation.x();
    node.pose.position.y = translation.y();
    node.pose.position.z = translation.z();
    node.pose.orientation.x = quaternion.x();
    node.pose.orientation.y = quaternion.y();
    node.pose.orientation.z = quaternion.z();
    node.pose.orientation.w = quaternion.w();
    pose_graph_nodes_.push_back(node);
  }
}

RosLoopClosureVisualizer::PoseGraphMsg
RosLoopClosureVisualizer::getPosegraphMsg() {
  PoseGraphMsg pose_graph;
  pose_graph.edges = odometry_edges_;
  pose_graph.edges.insert(pose_graph.edges.end(),
                          loop_closure_edges_.begin(),
                          loop_closure_edges_.end());
  pose_graph.nodes = pose_graph_nodes_;
  return pose_graph;
}

void RosLoopClosureVisualizer::publishPoseGraph(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  const Timestamp& ts = lcd_output->timestamp_;
  const gtsam::NonlinearFactorGraph& nfg = lcd_output->nfg_;
  const gtsam::Values& values = lcd_output->states_;
  updateNodesAndEdges(lcd_output->timestamp_map_, nfg, values);

  PoseGraphMsg graph = getPosegraphMsg();
  graph.header.stamp = rclcpp::Time(ts);
  graph.header.frame_id = map_frame_id_;
  posegraph_pub_->publish(graph);

  if (odometry_edges_.empty() || pose_graph_nodes_.size() < 2u) return;

  PoseGraphMsg incremental_graph;
  PoseGraphEdgeMsg last_odom_edge = odometry_edges_.back();
  last_odom_edge.header.stamp = rclcpp::Time(ts);
  last_odom_edge.type = PoseGraphEdgeMsg::ODOM;
  last_odom_edge.robot_from = robot_id_;
  last_odom_edge.robot_to = robot_id_;
  incremental_graph.edges.push_back(last_odom_edge);
  incremental_graph.nodes.push_back(
      pose_graph_nodes_[pose_graph_nodes_.size() - 2]);
  incremental_graph.nodes.push_back(pose_graph_nodes_.back());

  if (lcd_output->is_loop_closure_) {
    const gtsam::Pose3& lc_transform = lcd_output->relative_pose_;
    PoseGraphEdgeMsg last_lc_edge;
    const gtsam::Point3& translation = lc_transform.translation();
    const gtsam::Quaternion& quaternion =
        lc_transform.rotation().toQuaternion();
    last_lc_edge.key_from = lcd_output->id_match_;
    last_lc_edge.key_to = lcd_output->id_recent_;
    last_lc_edge.robot_from = robot_id_;
    last_lc_edge.robot_to = robot_id_;
    last_lc_edge.pose.position.x = translation.x();
    last_lc_edge.pose.position.y = translation.y();
    last_lc_edge.pose.position.z = translation.z();
    last_lc_edge.pose.orientation.x = quaternion.x();
    last_lc_edge.pose.orientation.y = quaternion.y();
    last_lc_edge.pose.orientation.z = quaternion.z();
    last_lc_edge.pose.orientation.w = quaternion.w();
    last_lc_edge.header.stamp = rclcpp::Time(ts);
    last_lc_edge.type = PoseGraphEdgeMsg::LOOPCLOSE;
    // updateNodesAndEdges() + updateRejectedEdges() (called above) have just
    // populated loop_closure_edges_ with the same LCD between-factor and its
    // gtsam noise-model covariance — but only for RPGO inliers (LCD detections
    // that survived RPGO's outlier rejector and made it into nfg). For
    // detections RPGO rejected, no matching entry exists and the lookup fails;
    // those edges have no factor to extract a noise model from, so we skip
    // publishing them rather than ship a covariance-less LOOPCLOSE edge that
    // a downstream factor-graph consumer (Hydra-RPGO) would treat as
    // infinite-confidence. /pose_graph_incremental and /pose_graph cumulative
    // therefore carry the same set of LOOPCLOSE edges (RPGO inliers only).
    bool found_inlier = false;
    for (const auto& edge : loop_closure_edges_) {
      if (edge.key_from == lcd_output->id_match_ &&
          edge.key_to == lcd_output->id_recent_ &&
          edge.type == PoseGraphEdgeMsg::LOOPCLOSE) {
        last_lc_edge.covariance = edge.covariance;
        found_inlier = true;
        break;
      }
    }
    if (found_inlier) {
      incremental_graph.edges.push_back(last_lc_edge);
    }
    // Do NOT push to loop_closure_edges_ — this LCD edge is already there
    // (with covariance) from updateRejectedEdges(). Pushing again created
    // a covariance-less duplicate that doubled LOOPCLOSE counts in the
    // cumulative pose_graph (e.g. 292 entries vs 146 unique on MH_03).
  }
  incremental_graph.header.stamp = rclcpp::Time(ts);
  incremental_graph.header.frame_id = map_frame_id_;
  posegraph_incremental_pub_->publish(incremental_graph);
}

void RosLoopClosureVisualizer::publishTf(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);
  const Timestamp& ts = lcd_output->timestamp_;
  const gtsam::Pose3& map_Pose_odom = lcd_output->Map_Pose_Odom_;

  geometry_msgs::msg::TransformStamped map_tf;
  map_tf.header.stamp = rclcpp::Time(ts);
  map_tf.header.frame_id = map_frame_id_;
  map_tf.child_frame_id = odom_frame_id_;
  utils::gtsamPoseToRosTf(map_Pose_odom, &map_tf.transform);
  tf_broadcaster_->sendTransform(map_tf);
}

void RosLoopClosureVisualizer::processBowQuery() {
  if (frames_.empty()) return;
  const size_t pose_id = frames_.size() - 1;
  if (bow_skip_num_ > 0 && (pose_id % bow_skip_num_) != 0) return;

  pose_graph_tools_msgs::msg::BowVector bow_vec_msg;
  for (const auto& kv : frames_.back().bow_vec_) {
    bow_vec_msg.word_ids.push_back(kv.first);
    bow_vec_msg.word_values.push_back(kv.second);
  }
  pose_graph_tools_msgs::msg::BowQuery bow_msg;
  bow_msg.robot_id = robot_id_;
  bow_msg.pose_id = pose_id;
  bow_msg.bow_vector = bow_vec_msg;
  for (uint16_t robot_id = 0; robot_id <= robot_id_; ++robot_id) {
    bow_queries_[robot_id].queries.push_back(bow_msg);
  }
}

void RosLoopClosureVisualizer::publishTimerCallback() {
  // Publish own queries.
  if (bow_queries_[robot_id_].queries.size() >=
      static_cast<size_t>(bow_batch_size_)) {
    bow_query_pub_->publish(bow_queries_[robot_id_]);
    bow_queries_[robot_id_].queries.clear();
  }

  uint16_t selected_robot_id = 0;
  size_t selected_batch_size = 0;
  for (uint16_t robot_id = 0; robot_id < robot_id_; ++robot_id) {
    if (bow_queries_[robot_id].queries.size() >= selected_batch_size) {
      selected_robot_id = robot_id;
      selected_batch_size = bow_queries_[robot_id].queries.size();
    }
  }
  if (selected_batch_size >= static_cast<size_t>(bow_batch_size_)) {
    RCLCPP_INFO(node_->get_logger(),
                "Published %zu BoW vectors to robot %hu.", selected_batch_size,
                selected_robot_id);
    bow_query_pub_->publish(bow_queries_[selected_robot_id]);
    bow_queries_[selected_robot_id].queries.clear();
  }

  if (new_frames_msg_.frames.size() >= 50) {
    vlc_frame_pub_->publish(new_frames_msg_);
    new_frames_msg_.frames.clear();
  }
}

void RosLoopClosureVisualizer::onVLCFrameQuery(
    const std::shared_ptr<VLCFrameQuerySrv::Request> request,
    std::shared_ptr<VLCFrameQuerySrv::Response> response) {
  CHECK(request);
  CHECK(response);
  CHECK_EQ(static_cast<uint16_t>(request->robot_id), robot_id_);
  response->frames.clear();
  for (const auto& pose_id : request->pose_ids) {
    VLCFrameMsg frame_msg;
    if (!getFrameMsg(static_cast<int>(pose_id), frame_msg)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "Requested frame " << pose_id << " does not exist!");
      continue;
    }
    response->frames.push_back(frame_msg);
  }
}

bool RosLoopClosureVisualizer::getFrameMsg(int pose_id,
                                            VLCFrameMsg& frame_msg) const {
  if (pose_id < 0 ||
      static_cast<size_t>(pose_id) >= frames_.size()) {
    return false;
  }
  const auto& frame = frames_[pose_id];

  frame_msg.robot_id = robot_id_;
  frame_msg.pose_id = pose_id;

  pcl::PointCloud<pcl::PointXYZ> versors;
  for (size_t i = 0; i < frame.keypoints_3d_.size(); ++i) {
    const gtsam::Vector3& v_ = frame.versors_[i];
    versors.push_back(pcl::PointXYZ(v_(0), v_(1), v_(2)));
    const gtsam::Vector3& p_ = frame.keypoints_3d_[i];
    if (p_.norm() < 1e-3) {
      frame_msg.depths.push_back(0);
    } else {
      frame_msg.depths.push_back(p_[2]);
    }
  }
  pcl::toROSMsg(versors, frame_msg.versors);

  cv_bridge::CvImage cv_img;
  cv_img.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  cv_img.image = frame.descriptors_mat_;
  cv_img.toImageMsg(frame_msg.descriptors_mat);
  return true;
}

}  // namespace VIO
