#pragma once
#include <exception>
#include <string>

namespace topic_config {
template <typename Builder, typename T>
class BuilderProp {
  bool _set = false;
  Builder& _builder;
  T _thing;

 public:
  BuilderProp(Builder& builder) : _builder(builder) {
  }
  BuilderProp(Builder& builder, const T& thing) : _builder(builder) {
    set(thing);
  }

  Builder& operator()(const T& thing) {
    set(thing);
    return _builder;
  }

  operator T() const {
    return T(get());
  }

  void operator=(const T& thing) {
    set(thing);
  }

  void unset() {
    _set = false;
  }

  bool is_set() const {
    return _set;
  }

  void assert_set() const {
    if (!is_set()) {
      throw std::runtime_error("Value not set!");
    }
  }

  void set(const T& thing) {
    _thing = thing;
    _set = true;
  }

  const T& get() const {
    assert_set();
    return _thing;
  }
};

/**
 * @brief Config for sending messages from a local topic to Robofleet
 * 
 * This creates a ROS subscriber to bridge local messages to Robofleet.
 * @tparam RosType the ROS message class being sent on the topic
 */
template <typename RosType>
struct SendLocalTopic {
  BuilderProp<SendLocalTopic, std::string> from{*this};
  BuilderProp<SendLocalTopic, std::string> to{*this};
  BuilderProp<SendLocalTopic, double> rate_limit_hz{*this};
  BuilderProp<SendLocalTopic, double> priority{*this, 1};
  BuilderProp<SendLocalTopic, bool> no_drop{*this, false};

  void assert_valid() const {
    from.assert_set();
    to.assert_set();
    priority.assert_set();
    if (priority <= 0) {
      throw std::runtime_error("Priority must be greater than 0!");
    }
    no_drop.assert_set();
  }
};

/**
 * @brief Config for receiving messages from a remote topic on Robofleet
 * 
 * This creates a ROS publisher to bridge remote messages to the robot.
 * @tparam RosType the ROS message class being sent on the topic
 */
template <typename RosType>
struct ReceiveRemoteTopic {
  BuilderProp<ReceiveRemoteTopic, std::string> from{*this};
  BuilderProp<ReceiveRemoteTopic, std::string> to{*this};

  void assert_valid() const {
    from.assert_set();
    to.assert_set();
  }
};
};