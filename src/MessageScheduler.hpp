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
  QString topic;
  int priority;

  friend bool operator<(const PrioritizedTopic& a, const PrioritizedTopic& b) {
    return a.priority > b.priority || a.topic < b.topic;
  }
};

/**
 * @brief Queues messages and schedules them on demand.
 * 
 * Messages are enqueued, and then later scheduled (via the scheduled signal)
 * when schedule() is called. The scheduling algorithm is approximately a token
 * bucket filter.
 */
class MessageScheduler : public QObject {
  Q_OBJECT
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = Clock::time_point;

  const double target_saturation = 0.7;
  double approx_bytes_per_sec = 0; // rate at which to generate bandwidth
  const double min_bytes_per_sec = 64; // assume at least this much bandwidth
  double bandwidth_bucket = 0; // currently accumulated bandwidth
  const double bucket_max_size_sec = 0.5; // how many sec worth of bandwidth can the bucket store? 
  TimePoint last_timestamp = Clock::now(); // when bandwidth was last generated

  std::deque<QByteArray> no_drop_queue;
  std::map<PrioritizedTopic, QByteArray> topic_queue;

  void update_bucket() {
    const TimePoint now = Clock::now();
    const double elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timestamp).count() / 1000.0;
    const double bytes_per_sec = std::max(min_bytes_per_sec, approx_bytes_per_sec);
    const double bucket_max_size = 500000; //bucket_max_size_sec * bytes_per_sec;

    bandwidth_bucket += elapsed_sec * bytes_per_sec * target_saturation;
    bandwidth_bucket = std::min(bandwidth_bucket, bucket_max_size);
    qDebug() << "BUCKET kB: " << (bandwidth_bucket / 1000.0);
    last_timestamp = now;
  }

  bool consume_bandwidth(int bytes, bool allow_deficit=false) {
    if (!allow_deficit && bandwidth_bucket < bytes) {
      return false;
    }
    bandwidth_bucket -= bytes;
    return true;
  }

public:
  MessageScheduler() {}

  Q_SIGNALS:
  void scheduled(const QByteArray& data);

public Q_SLOTS:
  /**
   * @brief Set the estimated bandwidth
   * @param bytes_per_sec estimated network bandwidth
   */
  void set_bandwidth(double bytes_per_sec) {
    approx_bytes_per_sec = bytes_per_sec;
  }

  void enqueue(const QString& topic, const QByteArray& data, int priority, bool no_drop) {
    if (no_drop) {
      no_drop_queue.push_back(data);
      if (no_drop_queue.size() > 2) {
        no_drop_queue.pop_front();
      }
    } else {
      PrioritizedTopic key;
      key.topic = topic;
      key.priority = priority;
      topic_queue[key] = data;
    }
    schedule();
  }

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority. Any messages on topics with 
   * the same priority will be sent together, temporarily ignoring throttling.
   */
  void schedule() {
    update_bucket();

    qDebug() << "tqs " << topic_queue.size() << "ndqs: " << no_drop_queue.size();

    while (!no_drop_queue.empty()) {
      const auto& next = no_drop_queue.front();
      qDebug() << "ndq item size" << next.size();
      if (consume_bandwidth(next.size())) {
        qDebug() << "\x1b[31mscheduled\x1b[0m nodrop";
        Q_EMIT scheduled(next);
        no_drop_queue.pop_front();
      } else {
        return;
      }
    }
    
    bool first_item = true;
    int prev_priority = 0;
    auto it = topic_queue.begin();
    while (it != topic_queue.end()) {
      const auto& key = it->first;
      const auto& data = it->second;
      const bool allow_deficit = !first_item && key.priority == prev_priority;
      prev_priority = key.priority;
      if (consume_bandwidth(data.size(), allow_deficit)) {
        qDebug() << "\x1b[32mscheduled\x1b[0m" << key.topic;
        Q_EMIT scheduled(data);
        it = topic_queue.erase(it);
        first_item = false;
      } else {
        return;
      }
    }
  }
};
