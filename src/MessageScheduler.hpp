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
#include "MessageSchedulerLib.hpp"
#include <functional>
#include "config.hpp"

/**
 * @brief Queues messages and schedules them on demand.
 *
 * Messages are enqueued, and then later scheduled (via the scheduled signal)
 * when schedule() is called.
 */
class MessageScheduler : public QObject {
  Q_OBJECT;
  MessageSchedulerLib<QByteArray>* ms;

 public:
  MessageScheduler() {
    setupSchedulerLib(config::max_queue_before_waiting);
  }

  void setupSchedulerLib(uint64_t mq) {
    std::function<void(const QByteArray&)> bound_callback(std::bind(&MessageScheduler::scheduling_callback, this, std::placeholders::_1));
    ms = new MessageSchedulerLib<QByteArray>(mq, bound_callback);
  }

  void scheduling_callback(const QByteArray& data) {
    Q_EMIT scheduled(data);
  }

 Q_SIGNALS:
  void scheduled(const QByteArray& data);

 public Q_SLOTS:
  void enqueue(
      const QString& topic, const QByteArray& data, double priority,
      bool no_drop) {
    ms->enqueue(topic.toUtf8().constData(), data, priority, no_drop);
  }

  /**
   * @brief Fire this to indicate that the network is free
   * Updates the counter for network backpressure
   */
  void backpressure_update(uint64_t message_index, uint64_t last_ponged_index) {
    ms->backpressure_update(message_index, last_ponged_index);
  }

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority.
   */
  void schedule() {
    ms->schedule();
  }
};
