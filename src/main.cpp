#include <ros/ros.h>

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
  ws_client.connect_ros_node(ros_node);

  return a.exec();
}
