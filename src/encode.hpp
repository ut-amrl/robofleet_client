#pragma once

#include <amrl_msgs/RobofleetStatus.h>
#include <flatbuffers/flatbuffers.h>
#include <nav_msgs/Odometry.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>

using FBB = flatbuffers::FlatBufferBuilder;
using MetadataOffset = flatbuffers::Offset<fb::MsgMetadata>;

// to add a new message type, specialize this template to encode the message
template <typename T>
static void encode(
    FBB& fbb, const T& msg, const std::string& msg_type,
    const std::string& topic);

template <>
void encode(
    FBB& fbb, const amrl_msgs::RobofleetStatus& msg,
    const std::string& msg_type, const std::string& topic) {
  auto metadata =
      fb::CreateMsgMetadataDirect(fbb, msg_type.c_str(), topic.c_str());

  auto rf_status = fb::amrl_msgs::CreateRobofleetStatusDirect(
      fbb,
      metadata,
      msg.status.c_str(),
      msg.is_ok,
      msg.battery_level,
      msg.location.c_str());

  fbb.Finish(rf_status);
}

template <>
void encode(
    FBB& fbb, const nav_msgs::Odometry& msg, const std::string& msg_type,
    const std::string& topic) {
  auto metadata =
      fb::CreateMsgMetadataDirect(fbb, msg_type.c_str(), topic.c_str());

  auto header_stamp =
      fb::RosTime(msg.header.stamp.toSec(), msg.header.stamp.toNSec());
  auto header = fb::std_msgs::CreateHeaderDirect(
      fbb, 0, msg.header.seq, &header_stamp, msg.header.frame_id.c_str());

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

template <>
void encode(
    FBB& fbb, const sensor_msgs::NavSatFix& msg, const std::string& msg_type,
    const std::string& topic) {
  auto metadata =
      fb::CreateMsgMetadataDirect(fbb, msg_type.c_str(), topic.c_str());

  auto header_stamp =
      fb::RosTime(msg.header.stamp.toSec(), msg.header.stamp.toNSec());
  auto header = fb::std_msgs::CreateHeaderDirect(
      fbb, 0, msg.header.seq, &header_stamp, msg.header.frame_id.c_str());

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
