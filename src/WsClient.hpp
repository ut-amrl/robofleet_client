#pragma once

#include <QObject>
#include <QTimer>
#include <QtWebSockets/QtWebSockets>
#include <cstdint>
#include <iostream>

#include "RosClientNode.hpp"
#include "config.hpp"

class WsClient : public QObject {
  Q_OBJECT;

  QUrl url;
  QWebSocket ws;
  QTimer recon_timer;
  QTimer ping_timer;

  // used to track which messages the server has received
  uint64_t last_ponged_index = 0;
  uint64_t msg_index = 0;

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

  void on_pong(qint64 elapsed_time, const QByteArray& payload) {
    uint64_t ponged_index = *reinterpret_cast<const uint64_t*>(payload.data());
    last_ponged_index = std::max(last_ponged_index, ponged_index);
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
    // don't buffer more data if we're waiting for old messages to be ACK'd
    if (config::wait_for_pongs &&
        msg_index - last_ponged_index > config::max_queue_before_waiting)
      return;

    ++msg_index;
    ws.sendBinaryMessage(data);
    send_ping();
  }

 Q_SIGNALS:
  void connected();
  void disconnected();
  void message_received(const QByteArray& data);

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
  }

  void connect_ros_node(const RosClientNode& ros_node) {
    // send
    QObject::connect(
        &ros_node,
        &RosClientNode::ros_message_encoded,
        this,
        &WsClient::send_message);
    // receive
    QObject::connect(
        this,
        &WsClient::message_received,
        &ros_node,
        &RosClientNode::decode_net_message);

    // startup
    QObject::connect(
        this,
        &WsClient::connected,
        &ros_node,
        &RosClientNode::subscribe_remote_msgs);

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
