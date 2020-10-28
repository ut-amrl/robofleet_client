#pragma once

#include <QObject>
#include <QTimer>
#include <QtWebSockets/QtWebSockets>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <chrono>

#include "config.hpp"

const static int ws_msg_overhead_bytes = 14;

class WsClient : public QObject {
  Q_OBJECT;
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = Clock::time_point;

  QUrl url;
  QWebSocket ws;
  QTimer recon_timer;
  QTimer ping_timer;

  // used to track which messages the server has received
  uint64_t last_ponged_index = 0;
  uint64_t msg_index = 0;
  std::unordered_map<uint64_t, int> size_by_index;
  std::unordered_map<uint64_t, TimePoint> time_by_index;

  double estimated_bytes_per_sec = 0;
  double estimate_alpha = 0.1; // how fast bps changes [0, 1]

 public Q_SLOTS:
  void on_error(QAbstractSocket::SocketError error) {
    std::cerr << "Websocket error: " << ws.errorString().toStdString()
              << std::endl;
  }

  void on_ssl_errors(const QList<QSslError>& errors) {
    std::cerr << "SSL Errors: " << std::endl;
    for (const QSslError& e : errors) {
      std::cerr << e.errorString().toStdString() << std::endl;
    }

    // WARNING: Never ignore SSL errors in production code.
    // The proper way to handle self-signed certificates is to add a custom root
    // to the CA store.
    // ws.ignoreSslErrors();
  }

  void on_connected() {
    std::cerr << "Websocket connected" << std::endl;
    Q_EMIT connected();
  }

  void on_disconnected() {
    std::cerr << "Websocket disconnected" << std::endl;
    Q_EMIT disconnected();
  }

  void on_binary_message(QByteArray data) {
    Q_EMIT message_received(data);
  }

  void on_pong(qint64 _elapsed_time, const QByteArray& payload) {
    uint64_t ponged_index = *reinterpret_cast<const uint64_t*>(payload.data());
    last_ponged_index = std::max(last_ponged_index, ponged_index);

    std::cout << "PONG " << ponged_index << " on " << msg_index << std::endl;

    // estimate bandwidth
    if (size_by_index.count(ponged_index)) {
      const double bytes = size_by_index[ponged_index];
      const double clock_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - time_by_index[ponged_index]).count();
      const double elapsed_ms = clock_elapsed - 40;
      const double instant_bytes_per_sec = bytes / (std::max(1.0, elapsed_ms) / 1000.0);
      
      estimated_bytes_per_sec = estimated_bytes_per_sec * (1.0 - estimate_alpha) + instant_bytes_per_sec * estimate_alpha;
      std::cout << "  elapsed sec " << (elapsed_ms / 1000.0) << std::endl;
      std::cout << "  est kBps    " << (estimated_bytes_per_sec / 1000.0) << std::endl;
      Q_EMIT bandwidth_estimated(estimated_bytes_per_sec);

      size_by_index.erase(ponged_index);
      time_by_index.erase(ponged_index);
    }
    if (msg_index - last_ponged_index < 2) {
      Q_EMIT network_unblocked();
    }
  }

  void reconnect() {
    std::cerr << "Websocket connecting to: " << url.toString().toStdString()
              << std::endl;
    ws.open(url);
  }

  void send_ping() {
    // reinterprets msg_index as a byte array;
    // endianness is not important since this data is just echoed back as is.
    const QByteArray msg_index_payload{
        reinterpret_cast<char*>(&msg_index), sizeof(msg_index)};
    ws.ping(msg_index_payload);
  }

  void send_message(const QByteArray& data) {
    ++msg_index;
    ws.sendBinaryMessage(data);
    size_by_index[msg_index] = data.size();
    time_by_index[msg_index] = Clock::now();
    send_ping();
  }

 Q_SIGNALS:
  void connected();
  void disconnected();
  void message_received(const QByteArray& data);
  void bandwidth_estimated(double approx_bytes_per_sec);
  void network_unblocked();

 public:
  WsClient(const QUrl& url) : url(url) {
    typedef void (QWebSocket::*errorSignal)(const QAbstractSocket::SocketError);
    typedef void (QWebSocket::*sslErrorsSignal)(const QList<QSslError>&);
    QObject::connect(
        &ws,
        static_cast<errorSignal>(&QWebSocket::error),
        this,
        &WsClient::on_error);
    QObject::connect(
        &ws,
        static_cast<sslErrorsSignal>(&QWebSocket::sslErrors),
        this,
        &WsClient::on_ssl_errors);
    QObject::connect(
        &ws, &QWebSocket::connected, this, &WsClient::on_connected);
    QObject::connect(
        &ws, &QWebSocket::disconnected, this, &WsClient::on_disconnected);
    QObject::connect(
        &ws,
        &QWebSocket::binaryMessageReceived,
        this,
        &WsClient::on_binary_message);
    QObject::connect(&ws, &QWebSocket::pong, this, &WsClient::on_pong);
    reconnect();

    // auto reconnect
    recon_timer.setSingleShot(true);
    QObject::connect(
        &recon_timer, &QTimer::timeout, this, &WsClient::reconnect);
    QObject::connect(this, &WsClient::disconnected, [&]() {
      recon_timer.start(std::chrono::milliseconds(2000).count());
    });

    // occasionally resend ping in case of network unreliability
    QObject::connect(
        &ping_timer, &QTimer::timeout, [&]() { this->send_ping(); });
    ping_timer.start(std::chrono::milliseconds(2000).count());
  }
};
