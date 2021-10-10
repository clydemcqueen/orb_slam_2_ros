/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "StereoNode.hpp"

#include <utility>

#include <camera_info_manager/camera_info_manager.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<StereoNode>("orb_slam2_stereo_node", options);

  node->init();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

StereoNode::StereoNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options)
{
  declare_parameter("stereo_side_by_side", rclcpp::ParameterValue(false));
  declare_parameter("left_info_url", rclcpp::ParameterValue(""));
  declare_parameter("right_info_url", rclcpp::ParameterValue(""));
}

void StereoNode::init()
{
  Node::init(ORB_SLAM2::System::STEREO);

  get_parameter("stereo_side_by_side", stereo_side_by_side_param_);
  get_parameter("left_info_url", left_info_url_param_);
  get_parameter("right_info_url", right_info_url_param_);

  if (stereo_side_by_side_param_) {
    RCLCPP_INFO(get_logger(), "Stereo side by side");

    camera_info_manager::CameraInfoManager left_camera_info_manager_(this, "stereo_left");
    camera_info_manager::CameraInfoManager right_camera_info_manager_(this, "stereo_right");

    if (left_camera_info_manager_.validateURL(left_info_url_param_)) {
      left_camera_info_manager_.loadCameraInfo(left_info_url_param_);
    } else {
      RCLCPP_ERROR(get_logger(), "Left camera info url '%s' is not valid, missing 'file://' prefix?", left_info_url_param_.c_str());
      return;
    }

    if (right_camera_info_manager_.validateURL(right_info_url_param_)) {
      right_camera_info_manager_.loadCameraInfo(right_info_url_param_);
    } else {
      RCLCPP_ERROR(get_logger(), "Right camera info url '%s' is not valid, missing 'file://' prefix?", right_info_url_param_.c_str());
      return;
    }

    stereo_model_.fromCameraInfo(
      left_camera_info_manager_.getCameraInfo(),
      right_camera_info_manager_.getCameraInfo());

    auto qos = subscribe_best_effort_param_ ?
      rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
      rclcpp::QoS{rclcpp::ServicesQoS()};

    side_by_side_subscriber_ = create_subscription<sensor_msgs::msg::Image>("/camera/side_by_side", qos,
      std::bind(&StereoNode::SideBySideCallback, this, std::placeholders::_1));
  } else {
    auto qos = subscribe_best_effort_param_ ?
      rmw_qos_profile_sensor_data :
      rmw_qos_profile_services_default;

    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      shared_from_this(), "/image_left/image_color_rect", qos);
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      shared_from_this(), "/image_right/image_color_rect", qos);

    sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&StereoNode::ImageCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  }
}

StereoNode::~StereoNode()
{
  delete sync_;
}

void StereoNode::SideBySideCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!isInitialized()) {
    RCLCPP_WARN(get_logger(), "Camera info not received, node has not been initialized!");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrBoth;
  try {
    cv_ptrBoth = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  int width = cv_ptrBoth->image.cols / 2;
  int height = cv_ptrBoth->image.rows;
  RCLCPP_INFO_ONCE(get_logger(), "L and R images are each %d x %d", width, height);

  // TODO simplify
  const cv::Mat left_roi(cv_ptrBoth->image, cv::Rect(0, 0, width, height));
  const cv::Mat right_roi(cv_ptrBoth->image, cv::Rect(width, 0, width, height));
  cv::Mat left_raw;
  left_roi.copyTo(left_raw);
  cv::Mat right_raw;
  right_roi.copyTo(right_raw);

  cv::Mat left_rect, right_rect;
  stereo_model_.left().rectifyImage(left_raw, left_rect);
  stereo_model_.right().rectifyImage(right_raw, right_rect);

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackStereo(left_rect, right_rect, current_frame_time_.seconds());

  Update();
}

void StereoNode::ImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msgLeft,
  const sensor_msgs::msg::Image::ConstSharedPtr & msgRight)
{
  if (!isInitialized()) {
    RCLCPP_WARN(get_logger(), "Camera info not received, node has not been initialized!");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
    cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  current_frame_time_ = msgLeft->header.stamp;

  orb_slam_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, current_frame_time_.seconds());

  Update();
}
