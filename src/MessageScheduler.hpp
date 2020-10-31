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
  // the message data
  QByteArray message;

  // is this message unsent and waiting?
  bool is_waiting = false; 

  // priority times number of times the message wasn't scheduled
  double total_prioritized_wait = 0;
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
      const auto key = PrioritizedTopic{topic, priority};
      topic_queue[key].message = data;
      topic_queue[key].is_waiting = true;
    }
    schedule();
  }

  /**
   * @brief Fire this to indicate that the network is free
   */
  void network_unblocked() {
    is_network_unblocked = true;
    qDebug() << "\x1b[35mUNBLOCKED\x1b[0m";
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

    // flush no-drop queue
    if (!no_drop_queue.empty()) {
      while (!no_drop_queue.empty()) {
        const auto& next = no_drop_queue.front();
        Q_EMIT scheduled(next);
        no_drop_queue.pop_front();
        qDebug() << "\x1b[33mnodrop\x1b[0m";
      }
      is_network_unblocked = false;
      return;
    }

    if (topic_queue.empty()) {
      return;
    }

    // update wait times for waiting messages.
    // select topic with the highest wait time.
    auto next = topic_queue.begin();
    for (auto it = topic_queue.begin(); it != topic_queue.end(); ++it) {
      const PrioritizedTopic& candidate_key = it->first;
      WaitingMessage& candidate_val = it->second;
      candidate_val.total_prioritized_wait += candidate_key.priority;

      qDebug() << candidate_key.topic << "\x1b[32mwait was\x1b[0m" << candidate_val.total_prioritized_wait;
      if (candidate_val.total_prioritized_wait > next->second.total_prioritized_wait) {
        next = it;
      }
    }

    if (next->second.is_waiting) {
      Q_EMIT scheduled(next->second.message);
      qDebug() << "\x1b[35mscheduled\x1b[0m" << next->first.topic;
      next->second.is_waiting = false;
      next->second.total_prioritized_wait = 0;
      is_network_unblocked = false;
    } else {
      qDebug() << "\x1b[31mwaiting\x1b[0m" << next->first.topic;
    }
  }
};
