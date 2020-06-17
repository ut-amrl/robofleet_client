#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/RobofleetStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <QTimer>
#include <QtCore/QCoreApplication>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "RosClientNode.hpp"
#include "WsClient.hpp"
#include "config.hpp"

int main(int argc, char** argv) {
  QCoreApplication a(argc, argv);
  ros::init(
      argc, argv, config::ros_node_name, ros::init_options::NoSigintHandler);

  // Websocket client
  WsClient ws_client{QString::fromStdString(config::host_url)};

  // Client ROS node
  RosClientNode ros_node;
  config::configure_msg_types(ros_node);

  // send
  QObject::connect(
      &ros_node,
      &RosClientNode::ros_message_encoded,
      &ws_client,
      &WsClient::send_message);
  // receive
  QObject::connect(
      &ws_client,
      &WsClient::message_received,
      &ros_node,
      &RosClientNode::decode_net_message);

  // auto reconnect
  QTimer recon_timer;
  recon_timer.setSingleShot(true);
  QObject::connect(
      &recon_timer, &QTimer::timeout, &ws_client, &WsClient::reconnect);
  QObject::connect(&ws_client, &WsClient::disconnected, [&]() {
    recon_timer.start(std::chrono::seconds(2));
  });

  return a.exec();
}
