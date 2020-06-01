#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

const static std::string node_name = "robofleet_client";

void test_callback(const std_msgs::String& msg) {
    std::cout << "Sending message: " << msg.data << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ros::Subscriber test_sub = n.subscribe("test", 8, test_callback);
    ros::spin();

    return 0;
}
