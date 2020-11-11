#pragma once

#include <amrl_msgs/ColoredArc2D.h>
#include <amrl_msgs/ColoredLine2D.h>
#include <amrl_msgs/ColoredPoint2D.h>
#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/PathVisualization.h>
#include <amrl_msgs/Point2D.h>
#include <amrl_msgs/Pose2Df.h>
#include <amrl_msgs/RobofleetStatus.h>
#include <amrl_msgs/RobofleetSubscription.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <flatbuffers/flatbuffers.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <schema_generated.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

// to add a new message type, specialize this template to decode the message
template <typename T>
static T decode(const void* const data);

// *** utility functions ***
/**
 * @brief Copy the ROS header from a flatbuffers Object API object src to a
 * ROS class object dst.
 *
 * @tparam Tsrc a generated flatbuffers Object API type
 * @tparam Tdst a ROS class type
 * @param src source
 * @param dst destination
 */
template <typename Tsrc, typename Tdst>
static void decode_header(const Tsrc& src, Tdst& dst) {
  dst.header.frame_id = src->header()->frame_id()->str();
  dst.header.seq = src->header()->seq();
  dst.header.stamp = ros::Time(
      src->header()->stamp()->secs(), src->header()->stamp()->nsecs());
}

/**
 * @brief Given a flatbuffers Object API object and a target ROS class type T,
 * call the decode<T>() function on the object's data.
 *
 * @tparam T target ROS class type
 * @tparam O a generated flatbuffers Object API type
 * @param src the flatbuffers object to decode
 * @return T the result of calling decode<T>() on src's data
 */
template <typename T, typename O>
static T decode_obj(const O* const src) {
  return decode<T>(flatbuffers::GetBufferStartFromRootPointer(src));
}

template <typename T, typename Vsrc, typename Vdst>
static void decode_obj_vector(
    const Vsrc* const src_vector_ptr, Vdst& dst_vector) {
  auto src = src_vector_ptr->begin();
  auto dst = dst_vector.begin();

  while (src != src_vector_ptr->end()) {
    *dst = decode_obj<T>(*src);
    ++src;
    ++dst;
  }
}

// *** specializations below ***

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
amrl_msgs::RobofleetSubscription decode(const void* const data) {
  const fb::amrl_msgs::RobofleetSubscription* src =
      flatbuffers::GetRoot<fb::amrl_msgs::RobofleetSubscription>(data);
  amrl_msgs::RobofleetSubscription dst;
  dst.action = src->action();
  dst.topic_regex = src->topic_regex()->str();
  return dst;
}

template <>
amrl_msgs::Point2D decode(const void* const data) {
  const fb::amrl_msgs::Point2D* src =
      flatbuffers::GetRoot<fb::amrl_msgs::Point2D>(data);
  amrl_msgs::Point2D dst;

  dst.x = src->x();
  dst.y = src->y();
  return dst;
}

template <>
amrl_msgs::Pose2Df decode(const void* const data) {
  const fb::amrl_msgs::Pose2Df* src =
      flatbuffers::GetRoot<fb::amrl_msgs::Pose2Df>(data);
  amrl_msgs::Pose2Df dst;

  dst.theta = src->theta();
  dst.x = src->x();
  dst.y = src->y();
  return dst;
}

template <>
amrl_msgs::ColoredArc2D decode(const void* const data) {
  const fb::amrl_msgs::ColoredArc2D* src =
      flatbuffers::GetRoot<fb::amrl_msgs::ColoredArc2D>(data);
  amrl_msgs::ColoredArc2D dst;

  dst.center = decode_obj<amrl_msgs::Point2D>(src->center());
  dst.color = src->color();
  dst.end_angle = src->end_angle();
  dst.radius = src->radius();
  dst.start_angle = src->start_angle();
  return dst;
}

template <>
amrl_msgs::ColoredLine2D decode(const void* const data) {
  const fb::amrl_msgs::ColoredLine2D* src =
      flatbuffers::GetRoot<fb::amrl_msgs::ColoredLine2D>(data);
  amrl_msgs::ColoredLine2D dst;

  dst.color = src->color();
  dst.p0 = decode_obj<amrl_msgs::Point2D>(src->p0());
  dst.p1 = decode_obj<amrl_msgs::Point2D>(src->p1());
  return dst;
}

template <>
amrl_msgs::ColoredPoint2D decode(const void* const data) {
  const fb::amrl_msgs::ColoredPoint2D* src =
      flatbuffers::GetRoot<fb::amrl_msgs::ColoredPoint2D>(data);
  amrl_msgs::ColoredPoint2D dst;

  dst.color = src->color();
  dst.point = decode_obj<amrl_msgs::Point2D>(src->point());
  return dst;
}

template <>
amrl_msgs::PathVisualization decode(const void* const data) {
  const fb::amrl_msgs::PathVisualization* src =
      flatbuffers::GetRoot<fb::amrl_msgs::PathVisualization>(data);
  amrl_msgs::PathVisualization dst;

  dst.clearance = src->clearance();
  dst.curvature = src->curvature();
  dst.distance = src->distance();
  return dst;
}

template <>
amrl_msgs::VisualizationMsg decode(const void* const data) {
  const fb::amrl_msgs::VisualizationMsg* src =
      flatbuffers::GetRoot<fb::amrl_msgs::VisualizationMsg>(data);
  amrl_msgs::VisualizationMsg dst;
  decode_header(src, dst);

  dst.ns = src->ns()->str();
  decode_obj_vector<amrl_msgs::ColoredArc2D>(src->arcs(), dst.arcs);
  decode_obj_vector<amrl_msgs::ColoredLine2D>(src->lines(), dst.lines);
  decode_obj_vector<amrl_msgs::PathVisualization>(
      src->path_options(), dst.path_options);
  decode_obj_vector<amrl_msgs::Pose2Df>(src->particles(), dst.particles);
  decode_obj_vector<amrl_msgs::ColoredPoint2D>(src->points(), dst.points);
  return dst;
}

template <>
amrl_msgs::Localization2DMsg decode(const void* const data) {
  const fb::amrl_msgs::Localization2DMsg* src =
      flatbuffers::GetRoot<fb::amrl_msgs::Localization2DMsg>(data);
  amrl_msgs::Localization2DMsg dst;
  decode_header(src, dst);
  dst.map = src->map()->str();
  dst.pose.x = src->pose()->x();
  dst.pose.y = src->pose()->y();
  dst.pose.theta = src->pose()->theta();
  return dst;
}

template <>
geometry_msgs::PoseStamped decode(const void* const data) {
  const fb::geometry_msgs::PoseStamped* src =
      flatbuffers::GetRoot<fb::geometry_msgs::PoseStamped>(data);
  geometry_msgs::PoseStamped dst;
  
  decode_header(src, dst);

  dst.pose.orientation.x = src->pose()->orientation()->x();
  dst.pose.orientation.y = src->pose()->orientation()->y();
  dst.pose.orientation.z = src->pose()->orientation()->z();
  dst.pose.orientation.w = src->pose()->orientation()->w();
  dst.pose.position.x = src->pose()->position()->x();
  dst.pose.position.y = src->pose()->position()->y();
  dst.pose.position.z = src->pose()->position()->z();

  return dst;
}


template <>
nav_msgs::Odometry decode(const void* const data) {
  const fb::nav_msgs::Odometry* src =
      flatbuffers::GetRoot<fb::nav_msgs::Odometry>(data);
  nav_msgs::Odometry dst;
  decode_header(src, dst);

  dst.child_frame_id = src->child_frame_id()->str();

  std::copy(
      src->pose()->covariance()->begin(),
      src->pose()->covariance()->end(),
      dst.pose.covariance.begin());
  dst.pose.pose.orientation.x = src->pose()->pose()->orientation()->x();
  dst.pose.pose.orientation.y = src->pose()->pose()->orientation()->y();
  dst.pose.pose.orientation.z = src->pose()->pose()->orientation()->z();
  dst.pose.pose.orientation.w = src->pose()->pose()->orientation()->w();
  dst.pose.pose.position.x = src->pose()->pose()->position()->x();
  dst.pose.pose.position.y = src->pose()->pose()->position()->y();
  dst.pose.pose.position.z = src->pose()->pose()->position()->z();

  std::copy(
      src->twist()->covariance()->begin(),
      src->twist()->covariance()->end(),
      dst.twist.covariance.begin());
  dst.twist.twist.angular.x = src->twist()->twist()->angular()->x();
  dst.twist.twist.angular.y = src->twist()->twist()->angular()->y();
  dst.twist.twist.angular.z = src->twist()->twist()->angular()->z();
  dst.twist.twist.linear.x = src->twist()->twist()->linear()->x();
  dst.twist.twist.linear.y = src->twist()->twist()->linear()->y();
  dst.twist.twist.linear.z = src->twist()->twist()->linear()->z();
  return dst;
}

template <>
sensor_msgs::LaserScan decode(const void* const data) {
  const fb::sensor_msgs::LaserScan* src =
      flatbuffers::GetRoot<fb::sensor_msgs::LaserScan>(data);
  sensor_msgs::LaserScan dst;
  decode_header(src, dst);
  dst.angle_increment = src->angle_increment();
  dst.angle_max = src->angle_max();
  dst.angle_min = src->angle_min();
  std::copy(
      src->intensities()->begin(),
      src->intensities()->end(),
      dst.intensities.begin());
  dst.range_max = src->range_max();
  dst.range_min = src->range_min();
  std::copy(src->ranges()->begin(), src->ranges()->end(), dst.ranges.begin());
  dst.scan_time = src->scan_time();
  dst.time_increment = src->time_increment();
  return dst;
}

template <>
sensor_msgs::NavSatFix decode(const void* const data) {
  const fb::sensor_msgs::NavSatFix* src =
      flatbuffers::GetRoot<fb::sensor_msgs::NavSatFix>(data);
  sensor_msgs::NavSatFix dst;
  decode_header(src, dst);
  dst.altitude = src->altitude();
  dst.latitude = src->latitude();
  dst.longitude = src->longitude();
  dst.position_covariance_type = src->position_covariance_type();
  dst.status.service = src->status()->service();
  dst.status.status = src->status()->status();
  return dst;
}

template <>
sensor_msgs::CompressedImage decode(const void* const data) {
  const fb::sensor_msgs::CompressedImage* src =
      flatbuffers::GetRoot<fb::sensor_msgs::CompressedImage>(data);
  sensor_msgs::CompressedImage dst;
  decode_header(src, dst);
  std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
  dst.format = src->format()->str();
  return dst;
}


template <>
sensor_msgs::PointField decode(const void* const data) {
  const fb::sensor_msgs::PointField* src =
      flatbuffers::GetRoot<fb::sensor_msgs::PointField>(data);
  sensor_msgs::PointField dst;
  dst.count = src->count();
  dst.name = src->name()->str();
  dst.offset = src->offset();
  dst.datatype = src->datatype();
  return dst;
}

template <>
sensor_msgs::PointCloud2 decode(const void* const data) {
  const fb::sensor_msgs::PointCloud2* src =
      flatbuffers::GetRoot<fb::sensor_msgs::PointCloud2>(data);
  sensor_msgs::PointCloud2 dst;
  decode_header(src, dst);
  std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
  decode_obj_vector<sensor_msgs::PointField>(src->fields(), dst.fields);
  dst.height = src->height();
  dst.width = src->width();
  dst.is_bigendian = src->is_bigendian();
  dst.is_dense = src->is_dense();
  dst.row_step = src->row_step();
  dst.point_step = src->point_step();
  return dst;
}

template <>
std_msgs::Header decode(const void* const data) {
  const fb::std_msgs::Header* src =
      flatbuffers::GetRoot<fb::std_msgs::Header>(data);
  std_msgs::Header dst;

  dst.frame_id = src->frame_id()->str();
  dst.seq = src->seq();
  dst.stamp = ros::Time(src->stamp()->secs(), src->stamp()->nsecs());
  return dst;
}
