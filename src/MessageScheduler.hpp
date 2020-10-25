#pragma once
#include <QObject>
#include <QString>
#include <QByteArray>
#include <QDebug>
#include <cstdint>
#include <unordered_map>
#include <deque>
#include <string>
#include <chrono>


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

  std::deque<std::tuple<QString, QByteArray>> queue;

  void update_bucket() {
    const TimePoint now = Clock::now();
    const double elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timestamp).count() / 1000.0;
    const double bytes_per_sec = std::max(min_bytes_per_sec, approx_bytes_per_sec);
    const double bucket_max_size = bucket_max_size_sec * bytes_per_sec;

    bandwidth_bucket += elapsed_sec * bytes_per_sec * target_saturation;
    bandwidth_bucket = std::min(bandwidth_bucket, bucket_max_size);
    qDebug() << "BUCKET kB: " << (bandwidth_bucket / 1000.0);
    last_timestamp = now;
  }

  bool consume_bandwidth(int bytes) {
    if (bandwidth_bucket <= 0) {
      return false;
    }
    bandwidth_bucket -= bytes;
    return true;
  }

public:
  MessageScheduler() {}

  Q_SIGNALS:
  void scheduled(const QString& topic, const QByteArray& data);

public Q_SLOTS:
  /**
   * @brief Set the estimated bandwidth
   * @param bytes_per_sec estimated network bandwidth
   */
  void set_bandwidth(double bytes_per_sec) {
    approx_bytes_per_sec = bytes_per_sec;
  }

  void enqueue(const QString& topic, const QByteArray& data) {
    if (queue.size() > 2) {
      queue.pop_front();
    }
    queue.push_back(std::make_tuple(topic, data));
    schedule();
  }

  /**
   * @brief Schedule messages now.
   */
  void schedule() {
    update_bucket();
    while (!queue.empty()) {
      const auto next = queue.front();
      const auto next_size_bytes = std::get<1>(next).size();
      if (consume_bandwidth(next_size_bytes)) {
        queue.pop_front();
        Q_EMIT scheduled(std::get<0>(next), std::get<1>(next));
        qDebug() << "\x1b[31mscheduled\x1b[0m";
      } else {
        break;
      }
    }
  }
};
