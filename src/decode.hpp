#pragma once

#include <flatbuffers/flatbuffers.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>
#include <amrl_msgs/RobofleetStatus.h>


// to add a new message type, specialize this template to decode the message
template <typename T>
static T decode(const void* const data);

template<>
amrl_msgs::RobofleetStatus decode(const void* const data) {
  const fb::amrl_msgs::RobofleetStatus* src = flatbuffers::GetRoot<fb::amrl_msgs::RobofleetStatus>(data);
  amrl_msgs::RobofleetStatus dst;
  dst.battery_level = src->battery_level();
  dst.is_ok = src->is_ok();
  dst.location = src->location()->str();
  dst.status = src->status()->str();
  return dst;
}

template<>
sensor_msgs::NavSatFix decode(const void* const data) {
  const fb::sensor_msgs::NavSatFix* src = flatbuffers::GetRoot<fb::sensor_msgs::NavSatFix>(data);
  sensor_msgs::NavSatFix dst;
  dst.altitude = src->altitude();
  dst.latitude = src->latitude();
  dst.longitude = src->longitude();
  dst.header.frame_id = src->header()->frame_id()->str();
  dst.header.seq = src->header()->seq();
  dst.header.stamp = ros::Time(src->header()->stamp()->secs(), src->header()->stamp()->nsecs());
  dst.position_covariance_type = src->position_covariance_type();
  dst.status.service = src->status()->service();
  dst.status.status = src->status()->status();
  return dst;
}
