#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <flatbuffers/flexbuffers.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "client.hpp"


const static std::string server_host = "localhost";
const static std::string server_port = "8080";
const static std::string node_name = "robofleet_client";

std::vector<uint8_t> encode(const std_msgs::String& msg) {
    flexbuffers::Builder fbb;
    fbb.Map([&](){
        fbb.String("data", msg.data);
    });
    fbb.Finish();
    return fbb.GetBuffer();
}

struct ROSHandler {
    std::shared_ptr<Client> client;

    explicit ROSHandler(std::shared_ptr<Client> client) : client(client) {}

    void test_callback(const std_msgs::String& msg) {
        if (!client->ws.is_open()) {
            std::cout << "ws connection not open" << std::endl;
            return;
        }
        std::cout << "transmitting " << msg.data << std::endl;
        client->ws.write(boost::asio::buffer(encode(msg)));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, node_name);

    // setup websocket client
    boost::asio::io_context ioc;
    auto client = std::make_shared<Client>(ioc);
    client->connect(server_host, server_port);
    
    ros::NodeHandle n;

    // set up subscriber to echo std_msgs/String messages out to websocket connection
    ROSHandler handler(client);
    ros::Subscriber test_sub = n.subscribe("test", 1, &ROSHandler::test_callback, &handler);
    client->receive_handler = [&](uint8_t const* data, std::size_t size) {
        std::cerr << "received message." << std::endl;
    };

    // run forever
    ros::spin();

    return 0;
}
