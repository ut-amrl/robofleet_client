#pragma once

#include <QObject>
#include <QtWebSockets/QtWebSockets>
#include <cstdint>
#include <iostream>

class WsClient : public QObject {
  Q_OBJECT;

  QUrl url;
  QWebSocket ws;
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
  }

  void reconnect() {
    std::cerr << "Websocket connecting to: " << url.toString().toStdString()
              << std::endl;
    ws.open(url);
  }

  void send_message(const QByteArray& data) {
    // don't buffer more bytes if we are still waiting on send
    if (bytes_buffered > 0)
      return;

    bytes_buffered = ws.sendBinaryMessage(data);
  }

 Q_SIGNALS:
  void connected();
  void disconnected();
  void message_received(const QByteArray& data);

 public:
  WsClient(const QUrl& url) : url(url) {
    typedef void (QWebSocket:: *errorSignal)(const QAbstractSocket::SocketError);
    typedef void (QWebSocket:: *sslErrorsSignal)(const QList<QSslError> &);
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
};
