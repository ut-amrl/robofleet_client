#pragma once

#include <QObject>
#include <QTimer>
#include <QtWebSockets/QtWebSockets>
#include <cstdint>
#include <iostream>
#include <unordered_set>

#include "RosClientNode.hpp"

class WsServer : public QObject {
  Q_OBJECT;
  QWebSocketServer* server;
  std::unordered_set<QWebSocket*> clients;

 public Q_SLOTS:
  void on_new_connection() {
    QWebSocket* socket = server->nextPendingConnection();

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

  void on_binary_message(QByteArray message) {
    QWebSocket* ws_sender = qobject_cast<QWebSocket*>(sender());
    Q_EMIT binary_message_received(message);
    Q_EMIT broadcast_message(message, ws_sender);
  }

  void on_socket_disconnected() {
    QWebSocket* client = qobject_cast<QWebSocket*>(sender());
    clients.erase(client);
    client->deleteLater();
    std::cerr << "Socket disconnected (" << clients.size() << " clients)"
              << std::endl;
  }

  void broadcast_message(
      const QByteArray& message, const QWebSocket* const ws_sender = nullptr) {
    for (QWebSocket* const client : clients) {
      if (client != ws_sender) {
        client->sendBinaryMessage(message);
      }
    }
  }

 Q_SIGNALS:
  void binary_message_received(const QByteArray& message);
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

  void connect_ros_node(const RosClientNode& ros_node) {
    // send
    // Need to use string-based connect to support default arg.
    // https://doc.qt.io/qt-5/signalsandslots-syntaxes.html
    // Trivial lambda-based version will not work because it is called from
    // another thread.
    QObject::connect(
        &ros_node,
        SIGNAL(ros_message_encoded(const QByteArray&)),
        this,
        SLOT(broadcast_message(const QByteArray&)));

    // receive
    QObject::connect(
        this,
        &WsServer::binary_message_received,
        &ros_node,
        &RosClientNode::decode_net_message);
  }
};
