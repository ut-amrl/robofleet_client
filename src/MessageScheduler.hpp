#pragma once
#include <QByteArray>
#include <QHash>
#include <QObject>
#include <QString>
#include <chrono>
#include <deque>
#include <unordered_map>

using SchedulerClock = std::chrono::high_resolution_clock;

struct WaitingMessage {
  // the message data
  QByteArray message;

  // is this message unsent and waiting?
  bool message_ready = false;

  double priority = 0;

  // when a message was last sent on this topic
  SchedulerClock::time_point last_send_time = SchedulerClock::now();

  // updated as now() - last_send_time, only when a message is ready
  std::chrono::duration<double> time_waiting =
      std::chrono::duration<double>::zero();
};

// for compatibility
struct QStringHash {
  std::size_t operator()(const QString& s) const {
    return qHash(s);
  }
};

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
  std::unordered_map<QString, WaitingMessage, QStringHash> topic_queue;

 public:
  MessageScheduler() {
  }

 Q_SIGNALS:
  void scheduled(const QByteArray& data);

 public Q_SLOTS:
  void enqueue(
      const QString& topic, const QByteArray& data, double priority,
      bool no_drop) {
    if (no_drop) {
      no_drop_queue.push_back(data);
    } else {
      topic_queue[topic].message = data;
      topic_queue[topic].message_ready = true;
      topic_queue[topic].priority = priority;
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
    const auto now = SchedulerClock::now();
    auto next = topic_queue.begin();
    for (auto it = topic_queue.begin(); it != topic_queue.end(); ++it) {
      const QString& candidate_topic = it->first;
      WaitingMessage& candidate = it->second;

      if (candidate.message_ready) {
        candidate.time_waiting =
            (now - candidate.last_send_time) * candidate.priority;
      }
      if (candidate.time_waiting > next->second.time_waiting) {
        next = it;
      }
    }

    if (next->second.message_ready) {
      Q_EMIT scheduled(next->second.message);
      next->second.message_ready = false;
      next->second.last_send_time = now;
      is_network_unblocked = false;
    }
  }
};
