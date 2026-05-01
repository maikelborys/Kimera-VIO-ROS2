/**
 * @file   RosDataProviderInterface.cpp
 */

#include "kimera_vio_ros/RosDataProviderInterface.hpp"

#include <fstream>

#include <cv_bridge/cv_bridge.hpp>
#include <glog/logging.h>
#include <sensor_msgs/image_encodings.hpp>

namespace VIO {

namespace {

bool getOrDeclareBool(rclcpp::Node* node,
                      const std::string& name,
                      bool default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<bool>(name, default_value);
  }
  return node->get_parameter(name).as_bool();
}

}  // namespace

RosDataProviderInterface::RosDataProviderInterface(rclcpp::Node* node,
                                                   const VioParams& vio_params)
    : DataProviderInterface(),
      node_(CHECK_NOTNULL(node)),
      vio_params_(vio_params),
      log_gt_data_(false),
      is_header_written_poses_vio_(false) {
  VLOG(1) << "Initializing RosDataProviderInterface.";
  log_gt_data_ = getOrDeclareBool(node_, "log_gt_data", false);
  if (log_gt_data_) {
    output_gt_poses_csv_.reset(new OfstreamWrapper("traj_gt.csv"));
  }
  if (VLOG_IS_ON(1)) printParsedParams();
}

RosDataProviderInterface::~RosDataProviderInterface() {
  VLOG(1) << "RosDataProviderInterface destructor called.";
}

cv::Mat RosDataProviderInterface::readRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  CHECK(img_msg);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    LOG(FATAL) << "cv_bridge exception: " << exception.what();
  }
  CHECK(cv_ptr);
  const cv::Mat img_const = cv_ptr->image;
  cv::Mat converted_img(img_const.size(), CV_8U);
  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGRA2GRAY);
    return converted_img;
  } else {
    CHECK(cv_ptr->encoding == sensor_msgs::image_encodings::MONO8 ||
          cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
        << "Expected MONO8 / 8UC1 / BGR8 / RGB8 image, got "
        << cv_ptr->encoding;
    return img_const;
  }
}

cv::Mat RosDataProviderInterface::readRosDepthImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    LOG(FATAL) << "cv_bridge exception: " << exception.what();
  }
  cv::Mat img_depth = cv_ptr->image;
  CHECK_EQ(img_depth.channels(), 1);
  if (img_depth.type() != CV_32FC1 && img_depth.type() != CV_16UC1) {
    LOG_FIRST_N(WARNING, 10)
        << "Converting depth image to CV_32FC1.";
    img_depth.convertTo(img_depth, CV_32FC1);
  }
  return img_depth;
}

void RosDataProviderInterface::logGtData(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odometry) {
  CHECK(odometry);
  if (!output_gt_poses_csv_) {
    LOG(ERROR) << "GT pose file not initialized!";
    return;
  }
  std::ofstream& output_stream = output_gt_poses_csv_->ofstream_;
  if (!is_header_written_poses_vio_) {
    output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,"
                  << "bgx,bgy,bgz,bax,bay,baz" << std::endl;
    is_header_written_poses_vio_ = true;
  }
  const auto stamp_ns = static_cast<int64_t>(odometry->header.stamp.sec) *
                            1000000000LL +
                        odometry->header.stamp.nanosec;
  output_stream << stamp_ns << ","
                << odometry->pose.pose.position.x << ","
                << odometry->pose.pose.position.y << ","
                << odometry->pose.pose.position.z << ","
                << odometry->pose.pose.orientation.w << ","
                << odometry->pose.pose.orientation.x << ","
                << odometry->pose.pose.orientation.y << ","
                << odometry->pose.pose.orientation.z << ","
                << odometry->twist.twist.linear.x << ","
                << odometry->twist.twist.linear.y << ","
                << odometry->twist.twist.linear.z
                << ",0,0,0,0,0,0" << std::endl;
}

void RosDataProviderInterface::printParsedParams() const {
  static constexpr int kSeparatorWidth = 40;
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Left camera info:";
  vio_params_.camera_params_.at(0).print();
  if (vio_params_.camera_params_.size() > 1) {
    LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Right camera info:";
    vio_params_.camera_params_.at(1).print();
  }
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Frontend params:";
  vio_params_.frontend_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - IMU params:";
  vio_params_.imu_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Backend params";
  vio_params_.backend_params_->print();
  LOG(INFO) << std::string(kSeparatorWidth, '=');
}

}  // namespace VIO
