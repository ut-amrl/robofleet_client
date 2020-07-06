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
#include "WebVizConstants.hpp"

namespace config {
// URL of robofleet_server instance (ignored in direct mode)
static const std::string host_url = "ws://localhost:8080";
// AMRL Robofleet server URL
// static const std::string host_url = "ws://10.0.0.1:8080";

// whether to run a Websocket server instead of a client, to bypass the need
// for a centralized instance of robofleet_server.
static const bool direct_mode = false;
// what port to serve on in direct mode
static const quint16 direct_mode_port = 8080;

// rate limiting:
// in normal mode, how many bytes to buffer for sending before dropping messages
static const qint64 max_send_buffer_bytes = 128000;
// in direct mode, maximum upload speed
static const quint64 direct_mode_bytes_per_sec = 2048000;

static const std::string ros_node_name = "robofleet_client";

/**
 * Configure all message types with which the client will interact.
 *
 * Each call to register_msg_type subscribes to the local ROS topic given as the
 * first argument and sends each message to the server with the new topic name
 * given as the second argument. You may call register_msg_type multiple times
 * for a given message type to send multiple topics. Calling it once is
 * sufficient to receive that message type from the server on any topic. Exactly
 * which messages you receive is controlled by your Robofleet subcriptions.
 *
 * Topic names beginning with a "/" are absolute ROS names; they will not be
 * prefixed with the current ROS namespace (robot name). To properly integrate
 * with Robofleet, you need to run this client with a ROS namespace representing
 * the robot's name. Most topics must be relative when sent to the server to
 * avoid name collisions between different robots.
 *
 * Here are some common use cases:
 * - To subscribe to a topic that is not namespaced, provide an absolute topic
 * name (beginning with "/").
 * - To subscribe to a namespaced topic, provide a relative topic name (not
 * beginning with "/").
 * - To send to a special webviz topic, make use of webviz_constants.
 * - To send to a custom topic, you should almost ALWAYS send to a relative
 * topic name to avoid name collisions.
 *
 */
static void configure_msg_types(RosClientNode& cn) {
  // Read all of the above documentation before modifying

  // must send to status topic to list robot in webviz
  cn.register_local_msg_type<amrl_msgs::RobofleetStatus>(
      "/status", webviz_constants::status_topic);

  // must send to subscriptions topic to receive messages from other robots
  cn.register_local_msg_type<amrl_msgs::RobofleetSubscription>(
      "/subscriptions", webviz_constants::subscriptions_topic);

 // Set up listeners for local messages for webviz
  cn.register_local_msg_type<amrl_msgs::Localization2DMsg>(
      "/localization", webviz_constants::localization_topic);

  cn.register_local_msg_type<nav_msgs::Odometry>(
      "/odometry/raw", webviz_constants::odometry_topic);

  cn.register_local_msg_type<sensor_msgs::LaserScan>(
      "/velodyne_2dscan", webviz_constants::lidar_2d_topic);

  cn.register_local_msg_type<sensor_msgs::CompressedImage>(
      "/stereo/left/image_raw/compressed", webviz_constants::left_image_topic);
  cn.register_local_msg_type<sensor_msgs::CompressedImage>(
      "/stereo/right/image_raw/compressed",
      webviz_constants::right_image_topic);

  cn.register_local_msg_type<amrl_msgs::VisualizationMsg>(
      "/visualization", webviz_constants::visualization_topic);

  // Set up listeners for remote messages
  cn.register_remote_msg_type<amrl_msgs::Pose2Df>(
      "move_base_simple/goal", "/move_base_simple/goal"
  );
  cn.register_remote_msg_type<amrl_msgs::Localization2DMsg>(
      "initialpose", "/initialpose"
  );

  // Add additional topics to subscribe and publish here.
}
}  // namespace config
