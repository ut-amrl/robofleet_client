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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <schema_generated.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

// to add a new message type, specialize this template to decode the message
// and...
template <typename Dst, typename Src>
static Dst decode(const Src* const src);
// specialize this struct to map ROS types to Flatbuffers types
template <typename RosType>
struct flatbuffers_type_for {
  typedef void type;
};

// *** utility functions ***
template <typename T, typename Vsrc, typename Vdst>
static void decode_vector(const Vsrc* const src_vector_ptr, Vdst& dst_vector) {
  dst_vector.resize(src_vector_ptr->size());
  auto src = src_vector_ptr->begin();
  auto dst = dst_vector.begin();

  while (src != src_vector_ptr->end()) {
    *dst = decode<T>(*src);
    ++src;
    ++dst;
  }
}

// *** specializations below ***

template <>
struct flatbuffers_type_for<std_msgs::Header> {
  typedef fb::std_msgs::Header type;
};
template <>
std_msgs::Header decode(const fb::std_msgs::Header* const src) {
  std_msgs::Header dst;

  dst.frame_id = src->frame_id()->str();
  dst.seq = src->seq();
  dst.stamp = ros::Time(src->stamp()->secs(), src->stamp()->nsecs());
  return dst;
}

template <>
struct flatbuffers_type_for<std_msgs::String> {
  typedef fb::std_msgs::String type;
};
template <>
std_msgs::String decode(const fb::std_msgs::String* const src) {
  std_msgs::String dst;
  dst.data = src->data()->str();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::RobofleetStatus> {
  typedef fb::amrl_msgs::RobofleetStatus type;
};
template <>
amrl_msgs::RobofleetStatus decode(
    const fb::amrl_msgs::RobofleetStatus* const src) {
  amrl_msgs::RobofleetStatus dst;
  dst.battery_level = src->battery_level();
  dst.is_ok = src->is_ok();
  dst.location = src->location()->str();
  dst.status = src->status()->str();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::RobofleetSubscription> {
  typedef fb::amrl_msgs::RobofleetSubscription type;
};
template <>
amrl_msgs::RobofleetSubscription decode(
    const fb::amrl_msgs::RobofleetSubscription* const src) {
  amrl_msgs::RobofleetSubscription dst;
  dst.action = src->action();
  dst.topic_regex = src->topic_regex()->str();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::Point2D> {
  typedef fb::amrl_msgs::Point2D type;
};
template <>
amrl_msgs::Point2D decode(const fb::amrl_msgs::Point2D* const src) {
  amrl_msgs::Point2D dst;

  dst.x = src->x();
  dst.y = src->y();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::Pose2Df> {
  typedef fb::amrl_msgs::Pose2Df type;
};
template <>
amrl_msgs::Pose2Df decode(const fb::amrl_msgs::Pose2Df* const src) {
  amrl_msgs::Pose2Df dst;

  dst.theta = src->theta();
  dst.x = src->x();
  dst.y = src->y();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::ColoredArc2D> {
  typedef fb::amrl_msgs::ColoredArc2D type;
};
template <>
amrl_msgs::ColoredArc2D decode(const fb::amrl_msgs::ColoredArc2D* const src) {
  amrl_msgs::ColoredArc2D dst;

  dst.center = decode<amrl_msgs::Point2D>(src->center());
  dst.color = src->color();
  dst.end_angle = src->end_angle();
  dst.radius = src->radius();
  dst.start_angle = src->start_angle();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::ColoredLine2D> {
  typedef fb::amrl_msgs::ColoredLine2D type;
};
template <>
amrl_msgs::ColoredLine2D decode(const fb::amrl_msgs::ColoredLine2D* const src) {
  amrl_msgs::ColoredLine2D dst;

  dst.color = src->color();
  dst.p0 = decode<amrl_msgs::Point2D>(src->p0());
  dst.p1 = decode<amrl_msgs::Point2D>(src->p1());
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::ColoredPoint2D> {
  typedef fb::amrl_msgs::ColoredPoint2D type;
};
template <>
amrl_msgs::ColoredPoint2D decode(
    const fb::amrl_msgs::ColoredPoint2D* const src) {
  amrl_msgs::ColoredPoint2D dst;

  dst.color = src->color();
  dst.point = decode<amrl_msgs::Point2D>(src->point());
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::PathVisualization> {
  typedef fb::amrl_msgs::PathVisualization type;
};
template <>
amrl_msgs::PathVisualization decode(
    const fb::amrl_msgs::PathVisualization* const src) {
  amrl_msgs::PathVisualization dst;

  dst.clearance = src->clearance();
  dst.curvature = src->curvature();
  dst.distance = src->distance();
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::VisualizationMsg> {
  typedef fb::amrl_msgs::VisualizationMsg type;
};
template <>
amrl_msgs::VisualizationMsg decode(
    const fb::amrl_msgs::VisualizationMsg* const src) {
  amrl_msgs::VisualizationMsg dst;
  dst.header = decode<std_msgs::Header>(src->header());

  dst.ns = src->ns()->str();
  decode_vector<amrl_msgs::ColoredArc2D>(src->arcs(), dst.arcs);
  decode_vector<amrl_msgs::ColoredLine2D>(src->lines(), dst.lines);
  decode_vector<amrl_msgs::PathVisualization>(
      src->path_options(), dst.path_options);
  decode_vector<amrl_msgs::Pose2Df>(src->particles(), dst.particles);
  decode_vector<amrl_msgs::ColoredPoint2D>(src->points(), dst.points);
  return dst;
}

template <>
struct flatbuffers_type_for<amrl_msgs::Localization2DMsg> {
  typedef fb::amrl_msgs::Localization2DMsg type;
};
template <>
amrl_msgs::Localization2DMsg decode(
    const fb::amrl_msgs::Localization2DMsg* const src) {
  amrl_msgs::Localization2DMsg dst;
  dst.header = decode<std_msgs::Header>(src->header());
  dst.map = src->map()->str();
  dst.pose.x = src->pose()->x();
  dst.pose.y = src->pose()->y();
  dst.pose.theta = src->pose()->theta();
  return dst;
}

template <>
struct flatbuffers_type_for<geometry_msgs::PoseStamped> {
  typedef fb::geometry_msgs::PoseStamped type;
};
template <>
geometry_msgs::PoseStamped decode(
    const fb::geometry_msgs::PoseStamped* const src) {
  geometry_msgs::PoseStamped dst;

  dst.header = decode<std_msgs::Header>(src->header());

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
struct flatbuffers_type_for<nav_msgs::Odometry> {
  typedef fb::nav_msgs::Odometry type;
};
template <>
nav_msgs::Odometry decode(const fb::nav_msgs::Odometry* const src) {
  nav_msgs::Odometry dst;
  dst.header = decode<std_msgs::Header>(src->header());

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
struct flatbuffers_type_for<sensor_msgs::LaserScan> {
  typedef fb::sensor_msgs::LaserScan type;
};
template <>
sensor_msgs::LaserScan decode(const fb::sensor_msgs::LaserScan* const src) {
  sensor_msgs::LaserScan dst;
  dst.header = decode<std_msgs::Header>(src->header());
  dst.angle_increment = src->angle_increment();
  dst.angle_max = src->angle_max();
  dst.angle_min = src->angle_min();
  dst.intensities.resize(src->intensities()->size());
  std::copy(
      src->intensities()->begin(),
      src->intensities()->end(),
      dst.intensities.begin());
  dst.range_max = src->range_max();
  dst.range_min = src->range_min();
  dst.ranges.resize(src->ranges()->size());
  std::copy(src->ranges()->begin(), src->ranges()->end(), dst.ranges.begin());
  dst.scan_time = src->scan_time();
  dst.time_increment = src->time_increment();
  return dst;
}

template <>
struct flatbuffers_type_for<sensor_msgs::NavSatFix> {
  typedef fb::sensor_msgs::NavSatFix type;
};
template <>
sensor_msgs::NavSatFix decode(const fb::sensor_msgs::NavSatFix* const src) {
  sensor_msgs::NavSatFix dst;
  dst.header = decode<std_msgs::Header>(src->header());
  dst.altitude = src->altitude();
  dst.latitude = src->latitude();
  dst.longitude = src->longitude();
  dst.position_covariance_type = src->position_covariance_type();
  dst.status.service = src->status()->service();
  dst.status.status = src->status()->status();
  return dst;
}

template <>
struct flatbuffers_type_for<sensor_msgs::CompressedImage> {
  typedef fb::sensor_msgs::CompressedImage type;
};
template <>
sensor_msgs::CompressedImage decode(
    const fb::sensor_msgs::CompressedImage* const src) {
  sensor_msgs::CompressedImage dst;
  dst.header = decode<std_msgs::Header>(src->header());
  dst.data.resize(src->data()->size());
  std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
  dst.format = src->format()->str();
  return dst;
}

template <>
struct flatbuffers_type_for<sensor_msgs::PointField> {
  typedef fb::sensor_msgs::PointField type;
};
template <>
sensor_msgs::PointField decode(const fb::sensor_msgs::PointField* const src) {
  sensor_msgs::PointField dst;
  dst.count = src->count();
  dst.name = src->name()->str();
  dst.offset = src->offset();
  dst.datatype = src->datatype();
  return dst;
}

template <>
struct flatbuffers_type_for<sensor_msgs::PointCloud2> {
  typedef fb::sensor_msgs::PointCloud2 type;
};
template <>
sensor_msgs::PointCloud2 decode(const fb::sensor_msgs::PointCloud2* const src) {
  sensor_msgs::PointCloud2 dst;
  dst.header = decode<std_msgs::Header>(src->header());
  dst.data.resize(src->data()->size());
  std::copy(src->data()->begin(), src->data()->end(), dst.data.begin());
  decode_vector<sensor_msgs::PointField>(src->fields(), dst.fields);
  dst.height = src->height();
  dst.width = src->width();
  dst.is_bigendian = src->is_bigendian();
  dst.is_dense = src->is_dense();
  dst.row_step = src->row_step();
  dst.point_step = src->point_step();
  return dst;
}
