#pragma once
#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/RobofleetStatus.h>
#include <amrl_msgs/RobofleetSubscription.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>

#include "RosClientNode.hpp"

namespace config {
static const std::string host_url = "wss://10.0.0.1:8080";
static const std::string ros_node_name = "robofleet_client";
static const std::string robot_name = "robot";

// Edit the configuration here to provide the correct topic names.
static const std::string status_topic = "status";
static const std::string subscription_topic = "subscriptions";
static const std::string localization_topic = "localization";
static const std::string odometry_topic = "odometry/raw";
static const std::string lidar_2d_topic = "velodyne_2dscan";
static const std::string left_image_topic = "stereo/left/image_raw/compressed";
static const std::string right_image_topic = "stereo/right/image_raw/compressed";

// All message types and subscribed topics must be enumerated here.
// Specializations must also be provided in encode.hpp and decode.hpp
static void configure_msg_types(RosClientNode& cn) {
  // These topics are used by webviz. Do not modify this section of the config
  cn.register_msg_type<amrl_msgs::RobofleetStatus>(status_topic, robot_name + "/" + "status");
  cn.register_msg_type<amrl_msgs::RobofleetSubscription>(subscription_topic, robot_name + "/" + "subscriptions");
  cn.register_msg_type<amrl_msgs::Localization2DMsg>(localization_topic, robot_name + "/" + "localization");
  cn.register_msg_type<nav_msgs::Odometry>(odometry_topic, robot_name + "/" + "odometry/raw");
  cn.register_msg_type<sensor_msgs::LaserScan>(lidar_2d_topic, robot_name + "/" + "velodyne_2dscan");
  cn.register_msg_type<sensor_msgs::CompressedImage>(left_image_topic, robot_name +  "/" +"stereo/left/image_raw/compressed");
  cn.register_msg_type<sensor_msgs::CompressedImage>(right_image_topic, robot_name +  "/" +"stereo/right/image_raw/compressed");

  // Add additional topics to subscribe and publish here.
}
}  // namespace config
