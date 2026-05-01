/**
 * @file   RosLoopClosureVisualizer.hpp
 * @brief  Slice C: ROS 2 port of the LCD output publisher.
 *
 * Publishes pose-graph + bag-of-words traffic on:
 *   - /optimized_trajectory       (nav_msgs/Path)
 *   - /optimized_odometry         (nav_msgs/Odometry)
 *   - /pose_graph                 (pose_graph_tools_msgs/PoseGraph)
 *   - /pose_graph_incremental     (pose_graph_tools_msgs/PoseGraph, deep queue)
 *   - /bow_query                  (pose_graph_tools_msgs/BowQueries)
 *   - /vlc_frames                 (pose_graph_tools_msgs/VLCFrames)
 *
 * Owns the map -> odom TF broadcast (RosVisualizer owns odom -> base_link).
 * Service: ~/vlc_frame_query (pose_graph_tools_msgs/VLCFrameQuery).
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pose_graph_tools_msgs/msg/bow_queries.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_edge.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_node.hpp>
#include <pose_graph_tools_msgs/msg/vlc_frame_msg.hpp>
#include <pose_graph_tools_msgs/msg/vlc_frames.hpp>
#include <pose_graph_tools_msgs/srv/vlc_frame_query.hpp>

#include <kimera-vio/loopclosure/LcdOutputPacket.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/utils/Macros.h>

namespace VIO {

class RosLoopClosureVisualizer {
 public:
  KIMERA_POINTER_TYPEDEFS(RosLoopClosureVisualizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosLoopClosureVisualizer);

  explicit RosLoopClosureVisualizer(rclcpp::Node* node);
  ~RosLoopClosureVisualizer() = default;

  /// Pipeline LCD callback entry point.
  void publishLcdOutput(const LcdOutput::ConstPtr& lcd_output);

 private:
  using PoseGraphMsg = pose_graph_tools_msgs::msg::PoseGraph;
  using PoseGraphEdgeMsg = pose_graph_tools_msgs::msg::PoseGraphEdge;
  using PoseGraphNodeMsg = pose_graph_tools_msgs::msg::PoseGraphNode;
  using BowQueriesMsg = pose_graph_tools_msgs::msg::BowQueries;
  using VLCFrameMsg = pose_graph_tools_msgs::msg::VLCFrameMsg;
  using VLCFramesMsg = pose_graph_tools_msgs::msg::VLCFrames;
  using VLCFrameQuerySrv = pose_graph_tools_msgs::srv::VLCFrameQuery;

  void publishTf(const LcdOutput::ConstPtr& lcd_output);
  void publishOptimizedTrajectory(const LcdOutput::ConstPtr& lcd_output);
  void publishPoseGraph(const LcdOutput::ConstPtr& lcd_output);

  void updateNodesAndEdges(const FrameIDTimestampMap& times,
                           const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values);
  void updateRejectedEdges();
  PoseGraphMsg getPosegraphMsg();

  void processBowQuery();
  void publishTimerCallback();
  void onVLCFrameQuery(
      const std::shared_ptr<VLCFrameQuerySrv::Request> request,
      std::shared_ptr<VLCFrameQuerySrv::Response> response);
  bool getFrameMsg(int pose_id, VLCFrameMsg& frame_msg) const;

  rclcpp::Node* node_;

  // Frame IDs (latched at construction).
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;

  uint16_t robot_id_;
  int bow_batch_size_;
  int bow_skip_num_;
  bool publish_vlc_frames_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PoseGraphMsg>::SharedPtr posegraph_pub_;
  rclcpp::Publisher<PoseGraphMsg>::SharedPtr posegraph_incremental_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<BowQueriesMsg>::SharedPtr bow_query_pub_;
  rclcpp::Publisher<VLCFramesMsg>::SharedPtr vlc_frame_pub_;

  rclcpp::Service<VLCFrameQuerySrv>::SharedPtr vlc_frame_server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<PoseGraphEdgeMsg> loop_closure_edges_;
  std::vector<PoseGraphEdgeMsg> odometry_edges_;
  std::vector<PoseGraphEdgeMsg> inlier_edges_;
  std::vector<PoseGraphNodeMsg> pose_graph_nodes_;

  struct lcd_frame {
    Landmarks keypoints_3d_;
    BearingVectors versors_;
    decltype(LcdOutput::bow_vec_) bow_vec_;
    cv::Mat descriptors_mat_;

    explicit lcd_frame(const LcdOutput& lcd_output)
        : keypoints_3d_(lcd_output.keypoints_3d_),
          versors_(lcd_output.versors_),
          bow_vec_(lcd_output.bow_vec_),
          descriptors_mat_(lcd_output.descriptors_mat_) {}
  };
  std::vector<lcd_frame> frames_;

  std::map<uint16_t, BowQueriesMsg> bow_queries_;
  VLCFramesMsg new_frames_msg_;
};

}  // namespace VIO
