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
static const std::string host_url = "ws://localhost:8080";
static const std::string ros_node_name = "robofleet_client";

static void configure_msg_types(RosClientNode& cn) {
  // All message types and subscribed topics must be enumerated here.
  // Specializations must also be provided in encode.hpp and decode.hpp
  cn.register_msg_type<amrl_msgs::RobofleetStatus>("status");
  cn.register_msg_type<amrl_msgs::RobofleetSubscription>("subscriptions");
  cn.register_msg_type<amrl_msgs::Localization2DMsg>("localization");
  cn.register_msg_type<nav_msgs::Odometry>("odometry/raw");
  cn.register_msg_type<sensor_msgs::LaserScan>("velodyne_2dscan");
  cn.register_msg_type<sensor_msgs::CompressedImage>(
      "stereo/left/image_raw/compressed");
}
}  // namespace config
