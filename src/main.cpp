#include <ros/ros.h>

#include <QTimer>
#include <QtCore/QCoreApplication>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "RosClientNode.hpp"
#include "WsClient.hpp"

static const QString server_host = "localhost";
static const int server_port = 8080;
static const std::string node_name = "robofleet_client";

void configure_msg_types(RosClientNode& cn) {
  // All message types and subscribed topics must be enumerated here.
  // Specializations must also be provided in encode.hpp and decode.hpp
  cn.register_msg_type<amrl_msgs::RobofleetStatus>("/test/status");
}

int main(int argc, char** argv) {
  QCoreApplication a(argc, argv);
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

  // Websocket client
  QUrl url;
  url.setScheme("ws");
  url.setHost(server_host);
  url.setPort(server_port);
  WsClient ws_client{url};

  // Client ROS node
  RosClientNode ros_node;
  configure_msg_types(ros_node);

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
  recon_timer.callOnTimeout(&ws_client, &WsClient::reconnect);
  QObject::connect(&ws_client, &WsClient::disconnected, [&]() {
    recon_timer.start(std::chrono::seconds(2));
  });

  return a.exec();
}
