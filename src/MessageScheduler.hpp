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
  const double priority;

  friend bool operator==(const PrioritizedTopic& a, const PrioritizedTopic& b) {
    return a.topic == b.topic && a.priority == b.priority;
  }
};

struct WaitingMessage {
  // the message data
  QByteArray message;

  // is this message unsent and waiting?
  bool is_waiting = false; 

  // sum of priority times wall-clock wait time
  double total_prioritized_wait = 0;
};

namespace std {
  template<> struct hash<PrioritizedTopic> {
    std::size_t operator()(const PrioritizedTopic& t) const noexcept {
      return std::hash<QString>()(t.topic) ^ std::hash<double>()(t.priority);
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
  using Clock = std::chrono::high_resolution_clock;
  bool is_network_unblocked = true;

  std::deque<QByteArray> no_drop_queue;
  std::unordered_map<PrioritizedTopic, WaitingMessage> topic_queue;
  Clock::time_point last_schedule_time;

public:
  MessageScheduler() : last_schedule_time(Clock::now()) {}

  Q_SIGNALS:
  void scheduled(const QByteArray& data);

public Q_SLOTS:
  void enqueue(const QString& topic, const QByteArray& data, double priority, bool no_drop) {
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

    // compute time since last schedule()
    const Clock::time_point now = Clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_schedule_time).count();
    const double elapsed_s = elapsed / 1000.0;
    last_schedule_time = now;

    // flush no-drop queue
    if (!no_drop_queue.empty()) {
      while (!no_drop_queue.empty()) {
        const auto& next = no_drop_queue.front();
        Q_EMIT scheduled(next);
        no_drop_queue.pop_front();
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
      candidate_val.total_prioritized_wait += candidate_key.priority * elapsed_s;

      if (candidate_val.total_prioritized_wait > next->second.total_prioritized_wait) {
        next = it;
      }
    }

    if (next->second.is_waiting) {
      Q_EMIT scheduled(next->second.message);
      next->second.is_waiting = false;
      next->second.total_prioritized_wait = 0;
      is_network_unblocked = false;
    }
  }
};
