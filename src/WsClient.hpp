#pragma once

#include <QObject>
#include <QtWebSockets/QtWebSockets>
#include <cstdint>
#include <iostream>

class WsClient : public QObject {
  Q_OBJECT;

  QWebSocket ws;

 public Q_SLOTS:
  void on_error(QAbstractSocket::SocketError error) {
    std::cerr << "Websocket error: " << ws.errorString().toStdString()
              << std::endl;
  }

  void on_connected() {
    std::cerr << "Websocket connected" << std::endl;
  }

  void on_disconnected() {
    std::cerr << "Websocket disconnected" << std::endl;
  }

  void on_binary_message(QByteArray data) {
    Q_EMIT message_received(data);
  }

  void send_message(const QByteArray& data) {
    ws.sendBinaryMessage(data);
  }

 Q_SIGNALS:
  void message_received(const QByteArray& data);

 public:
  WsClient(const QUrl& url) {
    QObject::connect(
        &ws, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error),
        this, &WsClient::on_error);
    QObject::connect(&ws, &QWebSocket::connected, this,
                     &WsClient::on_connected);
    QObject::connect(&ws, &QWebSocket::disconnected, this,
                     &WsClient::on_disconnected);
    QObject::connect(&ws, &QWebSocket::binaryMessageReceived, this,
                     &WsClient::on_binary_message);
    std::cerr << "Websocket connecting to: " << url.toString().toStdString()
              << std::endl;
    ws.open(url);
  }
};
