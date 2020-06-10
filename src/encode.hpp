#pragma once

#include <amrl_msgs/RobofleetStatus.h>
#include <flatbuffers/flatbuffers.h>
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
