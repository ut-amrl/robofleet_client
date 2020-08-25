#pragma once

#include <flatbuffers/flatbuffers.h>
#include <ros/ros.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>

#include <QObject>
#include <thread>
#include <unordered_map>
#include <chrono>

#include "decode.hpp"
#include "encode.hpp"

class RosClientNode : public QObject {
  Q_OBJECT

  ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
  ros::NodeHandle n;

  using TopicString = std::string;
  using MsgTypeString = std::string;
  std::vector<TopicString> pub_remote_topics;
  std::unordered_map<TopicString, ros::Subscriber> subs;
  std::unordered_map<TopicString, ros::Publisher> pubs;
  // Map from topic name to pair of <last publication time, publication interval?(sec)>
  std::unordered_map<TopicString, std::pair<std::chrono::time_point<std::chrono::high_resolution_clock>, double>> rate_limits;
  std::unordered_map<
      MsgTypeString, std::function<void(const QByteArray&, const TopicString&)>>
      pub_fns;

  /**
   * @brief Emit a ros_message_encoded() signal given a message and metadata.
   *
   * @tparam T type of msg
   * @param msg message to encode
   * @param msg_type type of the message; can be obtained using
   * ros::message_traits::DataType
   * @param from_topic topic on which message was received
   */
  template <typename T>
  void encode_ros_msg(
      const T& msg, const std::string& msg_type, const std::string& to_topic) {
    auto& rate_info = rate_limits[to_topic];
    const auto curr_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - rate_info.first);
    const double elapsed_sec = duration.count() / 1000.0;

    if (elapsed_sec > rate_info.second) {
      rate_info.first = curr_time;
      flatbuffers::FlatBufferBuilder fbb;
      auto metadata = encode_metadata(fbb, msg_type, to_topic);
      auto root_offset = encode<T>(fbb, msg, metadata);
      fbb.Finish(flatbuffers::Offset<void>(root_offset));
      Q_EMIT ros_message_encoded(QByteArray(
          reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()));
    }
  }

  template <typename T>
  void publish_ros_msg(
      const T& msg, const std::string& msg_type, const std::string& to_topic) {
    if (pubs.count(msg_type) == 0) {
      // gotcha: it's important that we only publish one type T for any given
      // msg_type! register_msg_type() handles this by checking for duplicate
      // msg_type registrations.
      pubs[msg_type] = n.advertise<T>(to_topic, 1);
      sleep(1); // we need to wait after setting up the publisher, or the first message gets dropped
    }

    pubs[msg_type].publish(msg);
  }

 Q_SIGNALS:
  void ros_message_encoded(const QByteArray& data);

 public Q_SLOTS:
  /**
   * @brief Attempt to decode an publish message data.
   *
   * Must call register_msg_type<T> before a message of type T can be decoded.
   * @param data the Flatbuffer-encoded message data
   */
  void decode_net_message(const QByteArray& data) {
    // extract metadata
    const fb::MsgWithMetadata* msg =
        flatbuffers::GetRoot<fb::MsgWithMetadata>(data.data());
    const std::string& msg_type = msg->__metadata()->type()->str();
    const std::string& topic = msg->__metadata()->topic()->str();
    std::cerr << "received " << msg_type << " message on " << topic
              << std::endl;

    // try to publish
    if (pub_fns.count(msg_type) == 0) {
      // if you get this unexpectedly, ensure that:
      // 1) you have registered the message type you intend to receive in your
      // configuration, and 2) you are sending a valid, fully-qualified message
      // type name (if you manually construct the flatbuffer)
      std::cerr << "ignoring message of unregistered type " << msg_type
                << std::endl;
      return;
    }
    pub_fns[msg_type](data, topic);
  }

  /**
   * @brief subscribe to remote messages that were specified in the config by
   * calling register_remote_msg_type. This function should run once a websocket
   * connection has been established
   */
  void subscribe_remote_msgs() {
    for (auto topic : pub_remote_topics) {
      sleep(1); // to avoid rate limiting issues. This only happens upon connection to the server so this time delay is no issue
      printf("Registering for remote subscription to topic %s\n", topic.c_str());
      // Now, subscribe to the appropriate remote message
      amrl_msgs::RobofleetSubscription sub_msg;
      sub_msg.action = amrl_msgs::RobofleetSubscription::ACTION_SUBSCRIBE;
      sub_msg.topic_regex = topic;
      encode_ros_msg<amrl_msgs::RobofleetSubscription>(
          sub_msg,
          ros::message_traits::DataType<amrl_msgs::RobofleetSubscription>()
              .value(),
          "/subscriptions");
    }
  }

 public:
  /**
   * @brief Set up pub/sub for a particular message type and topic.
   *
   * Creates a subscriber to the given topic, and emits ros_message_encoded()
   * signals for each message received.
   * @tparam T the ROS message type
   * @param from_topic the topic to subscribe to
   * @param to_topic the topic to publish to the server
   * @param max_publish_rate_hz the maximum number of messages per second to publish on this topic
   */
  template <typename T>
  void register_local_msg_type(
      const std::string& from_topic, const std::string& to_topic, uint8_t max_publish_rate_hz=10) {
    // apply remapping to encode full topic name
    const std::string full_from_topic = ros::names::resolve(from_topic);
    const std::string full_to_topic = ros::names::resolve(to_topic);
    const std::string& msg_type = ros::message_traits::DataType<T>().value();

    // Compute publish interval in seconds
    double publish_interval_sec = 1.0 / max_publish_rate_hz;

    if (subs.count(full_from_topic) > 0) {
      throw std::runtime_error(
          "Trying to register topic that is already registered. Topics must be "
          "unique.");
    }

    std::cerr << "registering " << msg_type << ": subscribing to "
              << full_from_topic << " and sending as " << full_to_topic
              << std::endl;

    rate_limits[full_to_topic] = std::make_pair(std::chrono::high_resolution_clock::now(), publish_interval_sec);

    // create subscription
    // have to use boost function because of how roscpp is implemented
    boost::function<void(T)> subscriber_handler =
        [this, msg_type, full_to_topic](T msg) {
          encode_ros_msg<T>(msg, msg_type, full_to_topic);
        };
    subs[full_from_topic] =
        n.subscribe<T>(full_from_topic, 1, subscriber_handler);
  }

  /**
   * @brief Set up remote publishing for a particular message type and topic
   * sent from the server.
   *
   * Sets up the client to publish to `to_topic` whenever it
   * recieves a message of type `from_topic` from the remote server.
   * @tparam T the ROS message type
   * @param from_topic the remote topic to expect
   * @param to_topic the local topic to publish to
   */
  template <typename T>
  void register_remote_msg_type(
      const std::string& from_topic, const std::string& to_topic) {
    // apply remapping to encode full topic name
    const std::string full_from_topic = ros::names::resolve(from_topic);
    const std::string full_to_topic = ros::names::resolve(to_topic);
    const std::string& msg_type = ros::message_traits::DataType<T>().value();

    if (subs.count(full_to_topic) > 0) {
      throw std::runtime_error(
          "Trying to publish to a topic that is registered as a subscription. "
          "This can create infinite feedback loops and is not allowed.");
    }

    std::cerr << "listening for remote topics of type " << msg_type
              << " on topic " << full_from_topic << " and publishing as "
              << full_to_topic << std::endl;
    
    // create function that will decode and publish a T message to any topic
    if (pub_fns.count(msg_type) == 0) {
      pub_fns[msg_type] = [this, full_to_topic](
                              const QByteArray& data,
                              const std::string& msg_type) {
        const T msg = decode<T>(data.data());
        publish_ros_msg<T>(msg, msg_type, full_to_topic);
      };
    }

    pub_remote_topics.push_back(full_from_topic);
  }

  RosClientNode() {
    // run forever
    spinner.start();
    std::cerr << "Started ROS Node" << std::endl;
  }
};
