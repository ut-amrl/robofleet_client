#include <ros/ros.h>

#include <QtCore/QCoreApplication>
#include <QObject>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "RosClientNode.hpp"
#include "WsClient.hpp"
#include "WsServer.hpp"
#include "MessageScheduler.hpp"
#include "config.hpp"

void connect_server(WsServer& ws_server, RosClientNode& ros_node, MessageScheduler& scheduler);
void connect_client(WsClient& ws_client, RosClientNode& ros_node, MessageScheduler& scheduler);

int main(int argc, char** argv) {
  QCoreApplication a(argc, argv);
  ros::init(
      argc, argv, config::ros_node_name, ros::init_options::NoSigintHandler);

  // Client ROS node
  RosClientNode ros_node;
  config::configure_msg_types(ros_node);

  MessageScheduler scheduler;

  if (config::direct_mode) {
    // Websocket server
    WsServer ws_server{config::direct_mode_port};
    connect_server(ws_server, ros_node, scheduler);
    return a.exec();
  } else {
    // Websocket client
    WsClient ws_client{QString::fromStdString(config::host_url)};
    connect_client(ws_client, ros_node, scheduler);
    return a.exec();
  }
}

void connect_client(WsClient& ws_client, RosClientNode& ros_node, MessageScheduler& scheduler) {
  // schedule messages
  QObject::connect(
      &ros_node,
      &RosClientNode::ros_message_encoded,
      &scheduler,
      &MessageScheduler::enqueue);
  // run scheduler
  QObject::connect(
    &ws_client,
    &WsClient::network_unblocked,
    &scheduler,
    &MessageScheduler::network_unblocked
  );
  // send scheduled message
  QObject::connect(
    &scheduler,
    &MessageScheduler::scheduled,
    [&ws_client](const QByteArray& data){ws_client.send_message(data);}
  );

  // receive
  QObject::connect(
      &ws_client,
      &WsClient::message_received,
      &ros_node,
      &RosClientNode::decode_net_message);

  // startup
  QObject::connect(
      &ws_client,
      &WsClient::connected,
      &ros_node,
      &RosClientNode::subscribe_remote_msgs);
}

void connect_server(WsServer& ws_server, RosClientNode& ros_node, MessageScheduler& scheduler) {
  // send
  // Need to use string-based connect to support default arg.
  // https://doc.qt.io/qt-5/signalsandslots-syntaxes.html
  // Trivial lambda-based version will not work because it is called from
  // another thread.
  QObject::connect(
      &ros_node,
      SIGNAL(ros_message_encoded(const QByteArray&)),
      &ws_server,
      SLOT(broadcast_message(const QByteArray&)));

  // receive
  QObject::connect(
      &ws_server,
      &WsServer::binary_message_received,
      &ros_node,
      &RosClientNode::decode_net_message);
}
