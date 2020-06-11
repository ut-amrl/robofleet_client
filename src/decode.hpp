#pragma once

#include <amrl_msgs/RobofleetStatus.h>
#include <flatbuffers/flatbuffers.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

// to add a new message type, specialize this template to decode the message
template <typename T>
static T decode(const void* const data);

template <>
amrl_msgs::RobofleetStatus decode(const void* const data) {
  const fb::amrl_msgs::RobofleetStatus* src =
      flatbuffers::GetRoot<fb::amrl_msgs::RobofleetStatus>(data);
  amrl_msgs::RobofleetStatus dst;
  dst.battery_level = src->battery_level();
  dst.is_ok = src->is_ok();
  dst.location = src->location()->str();
  dst.status = src->status()->str();
  return dst;
}

template <>
nav_msgs::Odometry decode(const void* const data) {
  const fb::nav_msgs::Odometry* src =
      flatbuffers::GetRoot<fb::nav_msgs::Odometry>(data);
  nav_msgs::Odometry dst;
  
  dst.header.frame_id = src->header()->frame_id()->str();
  dst.header.seq = src->header()->seq();
  dst.header.stamp = ros::Time(
      src->header()->stamp()->secs(), src->header()->stamp()->nsecs());
  
  dst.child_frame_id = src->child_frame_id()->str();
  
  std::copy(src->pose()->covariance()->begin(), src->pose()->covariance()->end(), dst.pose.covariance.begin());
  dst.pose.pose.orientation.x = src->pose()->pose()->orientation()->x();
  dst.pose.pose.orientation.y = src->pose()->pose()->orientation()->y();
  dst.pose.pose.orientation.z = src->pose()->pose()->orientation()->z();
  dst.pose.pose.orientation.w = src->pose()->pose()->orientation()->w();
  dst.pose.pose.position.x = src->pose()->pose()->position()->x();
  dst.pose.pose.position.y = src->pose()->pose()->position()->y();
  dst.pose.pose.position.z = src->pose()->pose()->position()->z();

  std::copy(src->twist()->covariance()->begin(), src->twist()->covariance()->end(), dst.twist.covariance.begin());
  dst.twist.twist.angular.x = src->twist()->twist()->angular()->x();
  dst.twist.twist.angular.y = src->twist()->twist()->angular()->y();
  dst.twist.twist.angular.z = src->twist()->twist()->angular()->z();
  dst.twist.twist.linear.x = src->twist()->twist()->linear()->x();
  dst.twist.twist.linear.y = src->twist()->twist()->linear()->y();
  dst.twist.twist.linear.z = src->twist()->twist()->linear()->z();
  return dst;
}

template <>
sensor_msgs::NavSatFix decode(const void* const data) {
  const fb::sensor_msgs::NavSatFix* src =
      flatbuffers::GetRoot<fb::sensor_msgs::NavSatFix>(data);
  sensor_msgs::NavSatFix dst;
  dst.altitude = src->altitude();
  dst.latitude = src->latitude();
  dst.longitude = src->longitude();
  dst.header.frame_id = src->header()->frame_id()->str();
  dst.header.seq = src->header()->seq();
  dst.header.stamp = ros::Time(
      src->header()->stamp()->secs(), src->header()->stamp()->nsecs());
  dst.position_covariance_type = src->position_covariance_type();
  dst.status.service = src->status()->service();
  dst.status.status = src->status()->status();
  return dst;
}
