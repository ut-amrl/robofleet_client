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

  bool waiting_for_send = false;
  qint64 bytes_buffered = 0;

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

  void on_bytes_written(qint64 bytes) {
    bytes_buffered -= bytes;

    // once buffer is empty, stop waiting
    if (bytes_buffered <= 0)
      waiting_for_send = false;
  }

  void reconnect() {
    std::cerr << "Websocket connecting to: " << url.toString().toStdString()
              << std::endl;
    ws.open(url);
  }

  void send_message(const QByteArray& data) {
    // don't buffer more bytes if we are still waiting on send
    if (waiting_for_send)
      return;

    bytes_buffered += ws.sendBinaryMessage(data);

    // once we hit maximum buffer size, wait for it to empty
    if (bytes_buffered > config::max_send_buffer_bytes)
      waiting_for_send = true;
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
    QObject::connect(
        &ws, &QWebSocket::bytesWritten, this, &WsClient::on_bytes_written);
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
  }
};
