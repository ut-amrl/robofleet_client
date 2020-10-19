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
#include "TopicConfig.hpp"
#include "WebVizConstants.hpp"

namespace config {
static const std::string ros_node_name = "robofleet_client";

// URL of robofleet_server instance (ignored in direct mode)
static const std::string host_url = "ws://localhost:8080";
// AMRL Robofleet server URL
// static const std::string host_url = "ws://10.0.0.1:8080";

/**
 * Anti-backpressure for normal mode.
 * Uses Websocket PING/PONG protocol to gauge when server has actually received
 * a message. If true, wait for PONGs before sending more messages.
 */
static const bool wait_for_pongs = true;

/**
 * If wait_for_acks, how many more messages to send before waiting for first
 * PONG? This can be set to a value greater than 0 to compensate for network
 * latency and fully saturate available bandwidth, but if it is set too high, it
 * could cause message lag.
 */
static const uint64_t max_queue_before_waiting = 1;

/**
 * Whether to run a Websocket server instead of a client, to bypass the need
 * for a centralized instance of robofleet_server.
 */
static const bool direct_mode = false;
static const quint16 direct_mode_port = 8080; // what port to serve on in direct mode
static const quint64 direct_mode_bytes_per_sec = 2048000; // avoid network backpressure in direct mode: sets maximum upload speed

/**
 * Configure all message types with which the client will interact.
 *
 * Topic names beginning with a "/" are absolute ROS names; they will not be
 * prefixed with the current ROS namespace (robot name). To properly integrate
 * with Robofleet, you need to run this client with a ROS namespace representing
 * the robot's name. Most topics must be relative (not beginning with "/") when
 * sent to the server to avoid name collisions between different robots.
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
  cn.configure(TopicConfig<amrl_msgs::RobofleetStatus>()
    .source(MessageSource::local)
    .from("/status")
    .to(webviz_constants::status_topic)
    .rate_limit_hz(1));

  // must send to subscriptions topic to receive messages from other robots
  // don't rate limit this topic.
  cn.configure(TopicConfig<amrl_msgs::RobofleetSubscription>()
    .source(MessageSource::local)
    .from("/subscriptions")
    .to(webviz_constants::subscriptions_topic)
    .rate_limit_hz(1));

  // send messages for webviz
  cn.configure(TopicConfig<amrl_msgs::Localization2DMsg>()
    .source(MessageSource::local)
    .from("/localization")
    .to(webviz_constants::localization_topic)
    .rate_limit_hz(10));

  cn.configure(TopicConfig<nav_msgs::Odometry>()
    .source(MessageSource::local)
    .from("/odometry/raw")
    .to(webviz_constants::odometry_topic)
    .rate_limit_hz(15));

  cn.configure(TopicConfig<sensor_msgs::LaserScan>()
    .source(MessageSource::local)
    .from("/velodyne_2dscan")
    .to(webviz_constants::lidar_2d_topic)
    .rate_limit_hz(15));

  cn.configure(TopicConfig<sensor_msgs::CompressedImage>()
    .source(MessageSource::local)
    .from("/stereo/left/image_raw/compressed")
    .to(webviz_constants::left_image_topic)
    .rate_limit_hz(10));
  cn.configure(TopicConfig<sensor_msgs::CompressedImage>()
    .source(MessageSource::local)
    .from("/stereo/right/image_raw/compressed")
    .to(webviz_constants::right_image_topic)
    .rate_limit_hz(10));

  cn.configure(TopicConfig<amrl_msgs::VisualizationMsg>()
    .source(MessageSource::local)
    .from("/visualization")
    .to(webviz_constants::visualization_topic)
    .rate_limit_hz(10));

  // receive remote commands
  cn.configure(TopicConfig<geometry_msgs::PoseStamped>()
    .source(MessageSource::remote)
    .from("move_base_simple/goal")
    .to("/move_base_simple/goal"));

  cn.configure(TopicConfig<amrl_msgs::Localization2DMsg>()
    .source(MessageSource::remote)
    .from("initialpose")
    .to("/initialpose"));

  // Add additional topics to subscribe and publish here.
}
}  // namespace config
