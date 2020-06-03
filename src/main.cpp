#include <cstdint>
#include <iostream>
#include <string>

#include <QtCore/QCoreApplication>
#include <ros/ros.h>

#include "RosClientNode.hpp"
#include "WsClient.hpp"


const static QString server_host = "localhost";
const static int server_port = 8080;
const static std::string node_name = "robofleet_client";


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

    QObject::connect(&ros_node, &RosClientNode::ros_message_encoded, &ws_client, &WsClient::send_message);
    QObject::connect(&ws_client, &WsClient::message_received, &ros_node, &RosClientNode::handle_message);

    return a.exec();
}
