#include <ros/ros.h>

#include <QtCore/QCoreApplication>
#include <cstdint>
#include <iostream>
#include <string>

#include "RosClientNode.hpp"
#include "WsClient.hpp"

static const QString server_host = "localhost";
static const int server_port = 8080;
static const std::string node_name = "robofleet_client";

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

  QObject::connect(&ros_node, &RosClientNode::ros_message_encoded, &ws_client,
                   &WsClient::send_message);
  QObject::connect(&ws_client, &WsClient::message_received, &ros_node,
                   &RosClientNode::handle_message);

  return a.exec();
}
