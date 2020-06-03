#pragma once

#include <thread>

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "encoding.hpp"

class RosClientNode : public QObject {
    Q_OBJECT

    ros::AsyncSpinner spinner = ros::AsyncSpinner(2);
    ros::NodeHandle n;
    ros::Subscriber test_sub;
    ros::Publisher test_pub;

    void test_callback(const std_msgs::String& msg) {
        std::cerr << "received ROS message: " << msg.data << std::endl;
        const std::vector<uint8_t> data = encode(msg);
        Q_EMIT ros_message_encoded(QByteArray(reinterpret_cast<const char*>(data.data()), data.size()));
    }

Q_SIGNALS:
    void ros_message_encoded(const QByteArray& data);

public Q_SLOTS:
    void handle_message(const QByteArray& data) {
        const auto msg = decode_string(reinterpret_cast<const uint8_t*>(data.data()), data.size());
        std::cerr << "received WS message: " << msg.data << std::endl;
        test_pub.publish(msg);
    }

public:
    RosClientNode() {
        test_sub = n.subscribe("test", 1, &RosClientNode::test_callback, this);
        test_pub = n.advertise<std_msgs::String>("test", 1);

        // run forever
        spinner.start();
        std::cerr << "Started ROS Node" << std::endl;
    }
};
