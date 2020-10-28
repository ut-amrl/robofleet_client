#pragma once
#include <QObject>
#include <QString>
#include <QByteArray>
#include <QDebug>
#include <cstdint>
#include <unordered_map>
#include <deque>
#include <map>
#include <string>
#include <chrono>

struct PrioritizedTopic {
  const QString topic;
  const int priority;

  friend bool operator==(const PrioritizedTopic& a, const PrioritizedTopic& b) {
    return a.topic == b.topic && a.priority == b.priority;
  }
};

struct WaitingMessage {
  QByteArray message;
  int64_t total_wait = 0;
};

namespace std {
  template<> struct hash<PrioritizedTopic> {
    std::size_t operator()(const PrioritizedTopic& t) const noexcept {
      return std::hash<QString>()(t.topic) ^ std::hash<int>()(t.priority);
    }
  };
}; // namespace std

/**
 * @brief Queues messages and schedules them on demand.
 * 
 * Messages are enqueued, and then later scheduled (via the scheduled signal)
 * when schedule() is called.
 */
class MessageScheduler : public QObject {
  Q_OBJECT
  bool is_network_unblocked = true;

  std::deque<QByteArray> no_drop_queue;
  std::unordered_map<PrioritizedTopic, WaitingMessage> topic_queue;

public:
  MessageScheduler() {}

  Q_SIGNALS:
  void scheduled(const QByteArray& data);

public Q_SLOTS:
  void enqueue(const QString& topic, const QByteArray& data, int priority, bool no_drop) {
    if (no_drop) {
      no_drop_queue.push_back(data);
    } else {
      topic_queue[PrioritizedTopic{topic, priority}].message = data;
    }
    schedule();
  }

  /**
   * @brief Fire this to indicate that the network is free
   */
  void network_unblocked() {
    is_network_unblocked = true;
    schedule();
  }

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority. 
   */
  void schedule() {
    if (!is_network_unblocked) {
      return;
    }
    is_network_unblocked = false;

    // flush no-drop queue
    if (!no_drop_queue.empty()) {
      while (!no_drop_queue.empty()) {
        const auto& next = no_drop_queue.front();
        Q_EMIT scheduled(next);
        no_drop_queue.pop_front();
      }
      return;
    }

    // select next regular message
    if (topic_queue.empty()) {
      return;
    }
    auto next_topic_pair = topic_queue.begin();
    for (auto it = topic_queue.begin(); it != topic_queue.end(); ++it) {
      auto& candidate_topic = it->first;
      auto& waiting_message = it->second;
      waiting_message.total_wait += candidate_topic.priority;
      if (waiting_message.total_wait > next_topic_pair->second.total_wait) {
        next_topic_pair = it;
      }
    }
    Q_EMIT scheduled(next_topic_pair->second.message);
    qDebug() << "\x1b[31mscheduled\x1b[0m" << next_topic_pair->first.topic;
    qDebug() << "\x1b[32mwait was\x1b[0m" << next_topic_pair->second.total_wait;
    topic_queue.erase(next_topic_pair);
  }
};
