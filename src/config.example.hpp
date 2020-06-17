#pragma once
#include <string>

#include "RosClientNode.hpp"

namespace config {
static const std::string host_url = "ws://localhost:8080";
static const std::string ros_node_name = "robofleet_client";

static void configure_msg_types(RosClientNode& cn) {
  // All message types and subscribed topics must be enumerated here.
  // Specializations must also be provided in encode.hpp and decode.hpp
  cn.register_msg_type<amrl_msgs::RobofleetStatus>("status");
  cn.register_msg_type<amrl_msgs::Localization2DMsg>("localization");
  cn.register_msg_type<nav_msgs::Odometry>("odometry/raw");
}
}  // namespace config
