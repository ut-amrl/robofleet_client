#pragma once

#include <QObject>
#include <QTimer>
#include <QtWebSockets/QtWebSockets>
#include <cstdint>
#include <iostream>
#include <unordered_set>

class WsServer : public QObject {
  Q_OBJECT;
  QWebSocketServer* server;
  std::unordered_set<QWebSocket*> clients;

 public Q_SLOTS:
  void on_new_connection() {
    QWebSocket* socket = server->nextPendingConnection();
    QObject::connect(
        socket,
        &QWebSocket::textMessageReceived,
        this,
        &WsServer::on_text_message);
    QObject::connect(
        socket,
        &QWebSocket::binaryMessageReceived,
        this,
        &WsServer::on_binary_message);
    QObject::connect(
        socket,
        &QWebSocket::disconnected,
        this,
        &WsServer::on_socket_disconnected);
    clients.emplace(socket);
    std::cerr << "New connection (" << clients.size() << " clients)"
              << std::endl;
  }

  void on_text_message(QString message) {
    // ignore text messages
  }

  void on_binary_message(QByteArray message) {
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
  }

  void on_socket_disconnected() {
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
    clients.erase(client);
    client->deleteLater();
    std::cerr << "Socket disconnected (" << clients.size() << " clients)"
              << std::endl;
  }

 Q_SIGNALS:
  void closed();

 public:
  WsServer(const quint16 port)
      : server(new QWebSocketServer(
            QStringLiteral("Robofleet"), QWebSocketServer::NonSecureMode)) {
    if (!server->listen(QHostAddress::Any, port)) {
      throw std::runtime_error(
          "Failed to listen on port " + std::to_string(port));
    }

    // handle client connections
    QObject::connect(
        server,
        &QWebSocketServer::newConnection,
        this,
        &WsServer::on_new_connection);
    QObject::connect(
        server, &QWebSocketServer::closed, this, &WsServer::closed);

    std::cerr << "WebSocket server listening on ws://*:" << server->serverPort()
              << std::endl;
  }

  ~WsServer() {
    server->close();
    qDeleteAll(clients.begin(), clients.end());
  }
};
