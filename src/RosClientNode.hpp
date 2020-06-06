#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <QObject>
#include <thread>

#include "encoding.hpp"

const static std::string nav_sat_fix_topic = "navsat/fix";

class RosClientNode : public QObject {
  Q_OBJECT

  ros::AsyncSpinner spinner = ros::AsyncSpinner(2);
  ros::NodeHandle n;
  ros::Subscriber test_sub;
  ros::Publisher test_pub;

  void nav_sat_fix_callback(const sensor_msgs::NavSatFix& msg) {
    std::cerr << "received ROS message" << std::endl;
    const std::vector<uint8_t> data = encode(msg);
    Q_EMIT ros_message_encoded(
        QByteArray(reinterpret_cast<const char*>(data.data()), data.size()));
  }

 Q_SIGNALS:
  void ros_message_encoded(const QByteArray& data);

 public Q_SLOTS:
  void handle_message(const QByteArray& data) {
    const auto msg = decode_nav_sat_fix(
        reinterpret_cast<const uint8_t*>(data.data()), data.size());
    std::cerr << "received WS message" << std::endl;
    test_pub.publish(msg);
  }

 public:
  RosClientNode() {
    test_sub = n.subscribe(nav_sat_fix_topic, 1, &RosClientNode::nav_sat_fix_callback, this);
    test_pub = n.advertise<sensor_msgs::NavSatFix>(nav_sat_fix_topic, 1);

    // run forever
    spinner.start();
    std::cerr << "Started ROS Node" << std::endl;
  }
};
