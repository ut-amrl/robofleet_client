#pragma once

#include <flatbuffers/flatbuffers.h>
#include <ros/ros.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>

#include <QObject>
#include <thread>
#include <unordered_map>

#include "decode.hpp"
#include "encode.hpp"

class RosClientNode : public QObject {
  Q_OBJECT

  ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
  ros::NodeHandle n;

  using TopicString = std::string;
  using MsgTypeString = std::string;
  std::unordered_map<TopicString, ros::Subscriber> subs;
  std::unordered_map<TopicString, ros::Publisher> pubs;
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
      const T& msg, const std::string& msg_type,
      const std::string& from_topic) {
    std::cerr << "encoding " << msg_type << " message from " << from_topic
              << std::endl;

    flatbuffers::FlatBufferBuilder fbb;
    encode<T>(fbb, msg, msg_type, from_topic);
    Q_EMIT ros_message_encoded(QByteArray(
        reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()));
  }

  template <typename T>
  void publish_ros_msg(const T& msg, const std::string& topic) {
    if (pubs.count(topic) == 0) {
      // gotcha: it's important that we only publish one type T for any given
      // topic! register_msg_type() handles this by checking for duplicate topic
      // registrations.
      pubs[topic] = n.advertise<T>(topic, 1);
    }
    pubs[topic].publish(msg);
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
      std::cerr << "ignoring message of type " << msg_type << std::endl;
      return;
    }
    pub_fns[msg_type](data, topic);
  }

 public:
  /**
   * @brief Set up pub/sub for a particular message type and topic.
   *
   * Creates a subscriber to the given topic, and emits ros_message_encoded()
   * signals for each message received.
   * Internally prepares to publish any message of type T from within the
   * decode_net_message() slot.
   * @tparam T the ROS message type
   * @param from_topic the topic to subscribe to
   */
  template <typename T>
  void register_msg_type(const std::string& from_topic) {
    // apply remapping to encode full topic name
    const std::string full_topic = ros::names::resolve(from_topic);
    const std::string& msg_type = ros::message_traits::DataType<T>().value();

    if (subs.count(full_topic) > 0) {
      throw std::runtime_error(
          "Trying to register topic that is already registered. Topics must be "
          "unique.");
    }

    std::cerr << "registering publisher for " << msg_type << " on topic "
              << full_topic << std::endl;

    // create subscription
    // have to use boost function because of how roscpp is implemented
    boost::function<void(T)> subscriber_handler =
        [this, msg_type, full_topic](T msg) {
          encode_ros_msg<T>(msg, msg_type, full_topic);
        };
    subs[full_topic] = n.subscribe<T>(full_topic, 1, subscriber_handler);

    // create function that will decode and publish a T message to any topic
    if (pub_fns.count(msg_type) == 0) {
      pub_fns[msg_type] =
          [this](const QByteArray& data, const std::string& topic) {
            const T msg = decode<T>(data.data());
            publish_ros_msg<T>(msg, topic);
          };
    }
  }

  RosClientNode() {
    // run forever
    spinner.start();
    std::cerr << "Started ROS Node" << std::endl;
  }
};
