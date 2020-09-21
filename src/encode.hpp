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
#include <geometry_msgs/PoseStamped.h>

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
  std::vector<flatbuffers::Offset<TEncoded>> dst(src.size());
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

// geometry_msgs/Point
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::Point& msg, const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreatePoint(fbb, metadata, msg.x, msg.y, msg.z).o;
}

// geometry_msgs/Quaternion
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::Quaternion& msg,
    const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreateQuaternion(
             fbb, metadata, msg.x, msg.y, msg.z, msg.w)
      .o;
}

// geometry_msgs/Pose
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::Pose& msg, const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreatePose(
             fbb,
             metadata,
             encode(fbb, msg.position, 0),
             encode(fbb, msg.orientation, 0))
      .o;
}

// geometry_msgs/PoseStamped
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::PoseStamped& msg, const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreatePoseStamped(
             fbb,
             metadata,
             encode(fbb, msg.header, 0),
             encode(fbb, msg.pose, 0))
      .o;
}

// geometry_msgs/PoseWithCovariance
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::PoseWithCovariance& msg,
    const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreatePoseWithCovariance(
             fbb,
             metadata,
             encode(fbb, msg.pose, 0),
             fbb.CreateVector(msg.covariance.data(), msg.covariance.size()))
      .o;
}

// geometry_msgs/Vector3
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::Vector3& msg,
    const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreateVector3(fbb, metadata, msg.x, msg.y, msg.z).o;
}

// geometry_msgs/Twist
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::Twist& msg, const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreateTwist(
             fbb,
             metadata,
             encode(fbb, msg.linear, 0),
             encode(fbb, msg.angular, 0))
      .o;
}

// geometry_msgs/TwistWithCovariance
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const geometry_msgs::TwistWithCovariance& msg,
    const MetadataOffset& metadata) {
  return fb::geometry_msgs::CreateTwistWithCovariance(
             fbb,
             metadata,
             encode(fbb, msg.twist, 0),
             fbb.CreateVector(msg.covariance.data(), msg.covariance.size()))
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

// amrl_msgs/ColoredPoint2D
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
  return fb::amrl_msgs::CreateVisualizationMsg(
             fbb,
             metadata,
             encode(fbb, msg.header, 0),
             fbb.CreateString(msg.ns),
             encode_vector<fb::amrl_msgs::Pose2Df>(fbb, 0, msg.particles),
             encode_vector<fb::amrl_msgs::PathVisualization>(
                 fbb, 0, msg.path_options),
             encode_vector<fb::amrl_msgs::ColoredPoint2D>(fbb, 0, msg.points),
             encode_vector<fb::amrl_msgs::ColoredLine2D>(fbb, 0, msg.lines),
             encode_vector<fb::amrl_msgs::ColoredArc2D>(fbb, 0, msg.arcs))
      .o;
}

// nav_msgs/Odometry
template <>
flatbuffers::uoffset_t encode(
    FBB& fbb, const nav_msgs::Odometry& msg, const MetadataOffset& metadata) {
  return fb::nav_msgs::CreateOdometryDirect(
             fbb,
             metadata,
             encode(fbb, msg.header, 0),
             msg.child_frame_id.c_str(),
             encode(fbb, msg.pose, 0),
             encode(fbb, msg.twist, 0))
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
