#pragma once
#include <exception>
#include <string>

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

enum class MessageSource { local, remote };

template <typename RosType>
struct TopicConfig {
  /**
   * @brief Whether the message is received from the local ROS master or the network.
   */
  BuilderProp<TopicConfig, MessageSource> source{*this};
  BuilderProp<TopicConfig, std::string> from{*this};
  BuilderProp<TopicConfig, std::string> to{*this};
  BuilderProp<TopicConfig, double> rate_limit_hz{*this};
  BuilderProp<TopicConfig, int> priority{*this, 1};
  BuilderProp<TopicConfig, bool> no_drop{*this, false};

  void assert_valid() const {
    source.assert_set();
    from.assert_set();
    to.assert_set();
    priority.assert_set();
    if (priority <= 0) {
      throw std::runtime_error("Priority must be greater than 0!");
    }
    no_drop.assert_set();
  }
};