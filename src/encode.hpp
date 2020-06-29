#pragma once

#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/RobofleetStatus.h>
#include <amrl_msgs/RobofleetSubscription.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <flatbuffers/flatbuffers.h>
#include <nav_msgs/Odometry.h>
#include <schema_generated.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

#include <algorithm>

using FBB = flatbuffers::FlatBufferBuilder;
using MetadataOffset = flatbuffers::Offset<fb::MsgMetadata>;

/**
 * @brief Encode a ROS message of type T into a flatbuffer object, placing it
 * into the flatbuffer builder fbb and returning its offset within the builder.
 *
 * This function should be specialized to support encoding various ROS message
 * types T.
 *
 * @tparam T the ROS message type to encode
 * @param fbb a Flatbuffer builder to store the encoded message in
 * @param msg the ROS message object to encode
 * @param metadata an optional metadata object, created with encode_metadata().
 * Pass 0 to let the metadata field be null.
 * @return flatbuffers::uoffset_t the offset of the encoded message within the
 * Flatbuffer being built in fbb.
 */
template <typename T>
static flatbuffers::uoffset_t encode(
    FBB& fbb, const T& msg, const MetadataOffset& metadata);

// *** utility functions ***
static flatbuffers::Offset<fb::MsgMetadata> encode_metadata(
    FBB& fbb, const std::string& msg_type, const std::string& topic) {
  return fb::CreateMsgMetadataDirect(fbb, msg_type.c_str(), topic.c_str());
}

/**
 * @brief Create a flatbuffer vector from a vector of ROS items by calling
 * encode<T>() on each item.
 *
 * @tparam TEncoded the target flatbuffers type to encode to
 * @tparam T the source ROS message type to encode from
 * @param fbb the flatbuffer builder in which to create the vector
 * @param metadata an optional metadata item to pass to encode() (pass 0 for
 * null)
 * @param src the vector of ROS messages to encode
 * @return flatbuffers::uoffset_t the offset of the vector within fbb
 */
template <typename TEncoded, typename T>
static flatbuffers::uoffset_t encode_vector(
    FBB& fbb, const MetadataOffset& metadata, std::vector<T> src) {
  std::vector<flatbuffers::Offset<TEncoded>> dst;
  std::transform(
      src.begin(), src.end(), dst.begin(), [&fbb, &metadata](const T& item) {
        return encode<T>(fbb, item, metadata);
      });
  return fbb.CreateVector(dst).o;
}

// *** specializations below ***

// std_msgs/Header
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const std_msgs::Header& msg, const MetadataOffset& metadata) {
  auto header_stamp = fb::RosTime(msg.stamp.toSec(), msg.stamp.toNSec());

  return fb::std_msgs::CreateHeaderDirect(
             fbb, 0, msg.seq, &header_stamp, msg.frame_id.c_str())
      .o;
}

// amrl_msgs/RobofleetStatus
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::RobofleetStatus& msg,
    const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateRobofleetStatusDirect(
             fbb,
             metadata,
             msg.status.c_str(),
             msg.is_ok,
             msg.battery_level,
             msg.location.c_str())
      .o;
}

// amrl_msgs/RobofleetSubscription
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::RobofleetSubscription& msg,
    const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateRobofleetSubscriptionDirect(
             fbb, metadata, msg.topic_regex.c_str(), msg.action)
      .o;
}

// amrl_msgs/Localization2DMsg
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::Localization2DMsg& msg,
    const MetadataOffset& metadata) {
  auto header = encode(fbb, msg.header, 0);

  auto pose = fb::amrl_msgs::CreatePose2Df(
      fbb, 0, msg.pose.x, msg.pose.y, msg.pose.theta);

  return fb::amrl_msgs::CreateLocalization2DMsgDirect(
             fbb, metadata, header, pose, msg.map.c_str())
      .o;
}

// amrl_msgs/Point2D
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::Point2D& msg, const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreatePoint2D(fbb, metadata, msg.x, msg.y).o;
}

// amrl_msgs/ColoredArc2D
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::ColoredArc2D& msg,
    const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateColoredArc2D(
             fbb,
             metadata,
             encode(fbb, msg.center, 0),
             msg.radius,
             msg.start_angle,
             msg.end_angle,
             msg.color)
      .o;
}

// amrl_msgs/ColoredLine2D
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::ColoredLine2D& msg,
    const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateColoredLine2D(
             fbb,
             metadata,
             encode(fbb, msg.p0, 0),
             encode(fbb, msg.p1, 0),
             msg.color)
      .o;
}

// amrl_msgs/Pose2Df
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::Pose2Df& msg, const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreatePose2Df(fbb, metadata, msg.x, msg.y, msg.theta).o;
}

// amrl_msgs/PathVisualization
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::PathVisualization& msg,
    const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreatePathVisualization(
             fbb, metadata, msg.curvature, msg.distance, msg.clearance)
      .o;
}

template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::ColoredPoint2D& msg,
    const MetadataOffset& metadata) {
  return fb::amrl_msgs::CreateColoredPoint2D(
             fbb, metadata, encode(fbb, msg.point, 0), msg.color)
      .o;
}

// amrl_msgs/VisualizationMsg
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const amrl_msgs::VisualizationMsg& msg,
    const MetadataOffset& metadata) {
  auto header = encode(fbb, msg.header, 0);
  auto ns = fbb.CreateString(msg.ns);
  auto arcs = encode_vector<fb::amrl_msgs::ColoredArc2D>(fbb, 0, msg.arcs);
  auto lines = encode_vector<fb::amrl_msgs::ColoredLine2D>(fbb, 0, msg.lines);
  auto particles = encode_vector<fb::amrl_msgs::Pose2Df>(fbb, 0, msg.particles);
  auto path_options =
      encode_vector<fb::amrl_msgs::PathVisualization>(fbb, 0, msg.path_options);
  auto points =
      encode_vector<fb::amrl_msgs::ColoredPoint2D>(fbb, 0, msg.points);

  return fb::amrl_msgs::CreateVisualizationMsg(
             fbb,
             metadata,
             header,
             ns,
             particles,
             path_options,
             points,
             lines,
             arcs)
      .o;
}

// nav_msgs/Odometry
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const nav_msgs::Odometry& msg, const MetadataOffset& metadata) {
  auto header = encode(fbb, msg.header, 0);

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

  return fb::nav_msgs::CreateOdometryDirect(
             fbb,
             metadata,
             header,
             msg.child_frame_id.c_str(),
             pose_with_cov,
             twist_with_cov)
      .o;
}

// sensor_msgs/LaserScan
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const sensor_msgs::LaserScan& msg,
    const MetadataOffset& metadata) {
  auto header = encode(fbb, msg.header, 0);

  auto ranges = fbb.CreateVector(msg.ranges.data(), msg.ranges.size());
  auto intensities =
      fbb.CreateVector(msg.intensities.data(), msg.intensities.size());

  return fb::sensor_msgs::CreateLaserScan(
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
             intensities)
      .o;
}

// sensor_msgs/NavSatFix
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const sensor_msgs::NavSatFix& msg,
    const MetadataOffset& metadata) {
  auto header = encode(fbb, msg.header, 0);

  auto status = fb::sensor_msgs::CreateNavSatStatus(
      fbb, 0, msg.status.service, msg.status.status);

  auto covariance = fbb.CreateVector(
      msg.position_covariance.data(), msg.position_covariance.size());

  return fb::sensor_msgs::CreateNavSatFix(
             fbb,
             metadata,
             header,
             status,
             msg.latitude,
             msg.longitude,
             msg.altitude,
             covariance,
             msg.position_covariance_type)
      .o;
}

// sensor_msgs/CompressedImage
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const sensor_msgs::CompressedImage& msg,
    const MetadataOffset& metadata) {
  auto header = encode(fbb, msg.header, 0);

  auto format = fbb.CreateString(msg.format);
  auto data = fbb.CreateVector(msg.data.data(), msg.data.size());

  return fb::sensor_msgs::CreateCompressedImage(
             fbb, metadata, header, format, data)
      .o;
}
