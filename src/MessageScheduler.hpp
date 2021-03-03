#pragma once
#include <QByteArray>
#include <QHash>
#include <QObject>
#include <QString>
#include <chrono>
#include <deque>
#include <iostream>
#include <unordered_map>
#include <vector>
#include "config.hpp"

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
  int network_backpressure_counter = 0;

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
   * Updates the counter for network backpressure
   */
  void backpressure_update(uint64_t message_index, uint64_t last_ponged_index) {
    network_backpressure_counter = message_index - last_ponged_index;
    schedule();
  }

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority.
   */
  void schedule() {
    if (network_backpressure_counter >= config::max_queue_before_waiting) {
      return;
    }

    // flush no-drop queue
    if (!no_drop_queue.empty()) {
      while (!no_drop_queue.empty()) {
        const auto& next = no_drop_queue.front();
        Q_EMIT scheduled(next);
        no_drop_queue.pop_front();
        network_backpressure_counter++;
      }
      if (network_backpressure_counter >= config::max_queue_before_waiting) {
        return;
      }
    }

    if (topic_queue.empty()) {
      return;
    }

    const auto now = SchedulerClock::now();

    // Collect candidates
    std::vector<WaitingMessage*> candidates;
    for (auto it = topic_queue.begin(); it != topic_queue.end(); ++it) {
      const QString& candidate_topic = it->first;
      WaitingMessage& candidate = it->second;

      if (candidate.message_ready) {
        candidate.time_waiting =
            (now - candidate.last_send_time) * candidate.priority;
        candidates.push_back(&candidate);
      }
    }
    int messages_to_send = std::min(
        (config::max_queue_before_waiting - network_backpressure_counter),
        candidates.size());

    // Sort candidates
    auto compare = [](WaitingMessage* lhs, WaitingMessage* rhs) {
      return lhs->time_waiting < rhs->time_waiting;
    };
    std::sort(candidates.begin(), candidates.end(), compare);

    // attampt to publish top-K candidates
    for (int cand_idx = 0; cand_idx < messages_to_send; cand_idx++) {
      if (candidates[cand_idx]->message_ready) {
        Q_EMIT scheduled(candidates[cand_idx]->message);
        candidates[cand_idx]->message_ready = false;
        candidates[cand_idx]->last_send_time = now;
        network_backpressure_counter++;
      }
    }
  }
};
