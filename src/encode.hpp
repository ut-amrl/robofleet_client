#pragma once

#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/RobofleetStatus.h>
#include <flatbuffers/flatbuffers.h>
#include <nav_msgs/Odometry.h>
#include <schema_generated.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

using FBB = flatbuffers::FlatBufferBuilder;
using MetadataOffset = flatbuffers::Offset<fb::MsgMetadata>;

// to add a new message type, specialize this template to encode the message
template <typename T>
static void encode(
    FBB& fbb, const T& msg, const std::string& msg_type,
    const std::string& topic);

// *** utility functions ***
static flatbuffers::Offset<fb::MsgMetadata> encode_metadata(
    FBB& fbb, const std::string& msg_type, const std::string& topic) {
  return fb::CreateMsgMetadataDirect(fbb, msg_type.c_str(), topic.c_str());
}
template <typename T>
static flatbuffers::Offset<fb::std_msgs::Header> encode_header(
    FBB& fbb, const T& msg) {
  auto header_stamp =
      fb::RosTime(msg.header.stamp.toSec(), msg.header.stamp.toNSec());
  return fb::std_msgs::CreateHeaderDirect(
      fbb, 0, msg.header.seq, &header_stamp, msg.header.frame_id.c_str());
}

// *** specializations below ***

// amrl_msgs/RobofleetStatus
template <>
void encode(
    FBB& fbb, const amrl_msgs::RobofleetStatus& msg,
    const std::string& msg_type, const std::string& topic) {
  auto metadata = encode_metadata(fbb, msg_type, topic);

  auto rf_status = fb::amrl_msgs::CreateRobofleetStatusDirect(
      fbb,
      metadata,
      msg.status.c_str(),
      msg.is_ok,
      msg.battery_level,
      msg.location.c_str());

  fbb.Finish(rf_status);
}

// amrl_msgs/Localization2DMsg
template <>
void encode(
    FBB& fbb, const amrl_msgs::Localization2DMsg& msg,
    const std::string& msg_type, const std::string& topic) {
  auto metadata = encode_metadata(fbb, msg_type, topic);
  auto header = encode_header(fbb, msg);

  auto pose = fb::amrl_msgs::CreatePose2Df(
      fbb, 0, msg.pose.x, msg.pose.y, msg.pose.theta);

  auto loc = fb::amrl_msgs::CreateLocalization2DMsgDirect(
      fbb, metadata, header, pose, msg.map.c_str());

  fbb.Finish(loc);
}

// nav_msgs/Odometry
template <>
void encode(
    FBB& fbb, const nav_msgs::Odometry& msg, const std::string& msg_type,
    const std::string& topic) {
  auto metadata = encode_metadata(fbb, msg_type, topic);
  auto header = encode_header(fbb, msg);

  // pose
  auto pose_position = fb::geometry_msgs::CreatePoint(
      fbb,
      0,
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z);
  auto pose_orientation = fb::geometry_msgs::CreateQuaternion(
      fbb,
      0,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
  auto pose =
      fb::geometry_msgs::CreatePose(fbb, 0, pose_position, pose_orientation);
  auto pose_with_cov = fb::geometry_msgs::CreatePoseWithCovariance(
      fbb,
      0,
      pose,
      fbb.CreateVector(msg.pose.covariance.data(), msg.pose.covariance.size()));

  // twist
  auto twist_linear = fb::geometry_msgs::CreateVector3(
      fbb,
      0,
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z);
  auto twist_angular = fb::geometry_msgs::CreateVector3(
      fbb,
      0,
      msg.twist.twist.angular.x,
      msg.twist.twist.angular.y,
      msg.twist.twist.angular.z);
  auto twist =
      fb::geometry_msgs::CreateTwist(fbb, 0, twist_linear, twist_angular);
  auto twist_with_cov = fb::geometry_msgs::CreateTwistWithCovariance(
      fbb,
      0,
      twist,
      fbb.CreateVector(
          msg.twist.covariance.data(), msg.twist.covariance.size()));

  auto odom = fb::nav_msgs::CreateOdometryDirect(
      fbb,
      metadata,
      header,
      msg.child_frame_id.c_str(),
      pose_with_cov,
      twist_with_cov);

  fbb.Finish(odom);
}

// sensor_msgs/LaserScan
template <>
void encode(
    FBB& fbb, const sensor_msgs::LaserScan& msg, const std::string& msg_type,
    const std::string& topic) {
  auto metadata = encode_metadata(fbb, msg_type, topic);
  auto header = encode_header(fbb, msg);

  auto ranges = fbb.CreateVector(msg.ranges.data(), msg.ranges.size());
  auto intensities =
      fbb.CreateVector(msg.intensities.data(), msg.intensities.size());

  auto scan = fb::sensor_msgs::CreateLaserScan(
      fbb,
      metadata,
      header,
      msg.angle_min,
      msg.angle_max,
      msg.angle_increment,
      msg.time_increment,
      msg.scan_time,
      msg.range_min,
      msg.range_max,
      ranges,
      intensities);

  fbb.Finish(scan);
}

// sensor_msgs/NavSatFix
template <>
void encode(
    FBB& fbb, const sensor_msgs::NavSatFix& msg, const std::string& msg_type,
    const std::string& topic) {
  auto metadata = encode_metadata(fbb, msg_type, topic);
  auto header = encode_header(fbb, msg);

  auto status = fb::sensor_msgs::CreateNavSatStatus(
      fbb, 0, msg.status.service, msg.status.status);

  auto covariance = fbb.CreateVector(
      msg.position_covariance.data(), msg.position_covariance.size());

  auto nav_sat_fix = fb::sensor_msgs::CreateNavSatFix(
      fbb,
      metadata,
      header,
      status,
      msg.latitude,
      msg.longitude,
      msg.altitude,
      covariance,
      msg.position_covariance_type);

  fbb.Finish(nav_sat_fix);
}

// sensor_msgs/CompressedImage
template <>
void encode(
    FBB& fbb, const sensor_msgs::CompressedImage& msg,
    const std::string& msg_type, const std::string& topic) {
  auto metadata = encode_metadata(fbb, msg_type, topic);
  auto header = encode_header(fbb, msg);

  auto format = fbb.CreateString(msg.format);
  auto data = fbb.CreateVector(msg.data.data(), msg.data.size());

  auto img = fb::sensor_msgs::CreateCompressedImage(
      fbb, metadata, header, format, data);

  fbb.Finish(img);
}
