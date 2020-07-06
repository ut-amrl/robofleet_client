#include <ros/ros.h>

#include <QtCore/QCoreApplication>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "RosClientNode.hpp"
#include "WsClient.hpp"
#include "WsServer.hpp"
#include "config.hpp"

int main(int argc, char** argv) {
  QCoreApplication a(argc, argv);
  ros::init(
      argc, argv, config::ros_node_name, ros::init_options::NoSigintHandler);

  // Client ROS node
  RosClientNode ros_node;
  config::configure_msg_types(ros_node);

  if (config::direct_mode) {
    // Websocket server
    WsServer ws_server{config::direct_mode_port};

    return a.exec();
  } else {
    // Websocket client
    WsClient ws_client{QString::fromStdString(config::host_url)};
    ws_client.connect_ros_node(ros_node);
    return a.exec();
  }
}
