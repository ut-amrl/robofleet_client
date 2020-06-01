#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <flatbuffers/flexbuffers.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

const static std::string node_name = "robofleet_client";

std::vector<uint8_t> encode(const std_msgs::String& msg) {
    flexbuffers::Builder fbb;
    fbb.Map([&](){
        fbb.String("data", msg.data);
    });
    fbb.Finish();
    return fbb.GetBuffer();
}

void test_callback(const std_msgs::String& msg) {
    std::vector<uint8_t> buffer = encode(msg);
    std::cout << "Sending message length: " << buffer.size() << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ros::Subscriber test_sub = n.subscribe("test", 8, test_callback);
    ros::spin();

    return 0;
}
