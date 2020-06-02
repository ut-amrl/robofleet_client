#pragma once

#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

namespace beast = boost::beast;
namespace websocket = beast::websocket; 

class Client : public std::enable_shared_from_this<Client> {
    using tcp = boost::asio::ip::tcp;    

    tcp::resolver resolver;
    std::string host_header;
    std::unique_ptr<std::thread> thread;

    void read_loop() {
        try {
            while (true) {
                beast::flat_buffer buffer;
                std::cerr << "reading..." << std::endl;
                ws.read(buffer);
                receive_handler(
                    boost::asio::buffer_cast<uint8_t const*>(
                        beast::buffers_front(buffer.data())
                    ),
                    boost::asio::buffer_size(buffer.data())
                );
            }
        } catch (beast::system_error const& se) {
            // This indicates that the session was closed
            if (se.code() != websocket::error::closed)
                throw se;
        } 
    }

public:
    beast::websocket::stream<beast::tcp_stream> ws;
    std::function<void(uint8_t const*, std::size_t)> receive_handler;

    explicit Client(boost::asio::io_context& ioc)
        : resolver(boost::asio::make_strand(ioc)),
          ws(boost::asio::make_strand(ioc)) {}
    
    ~Client() {
        thread->join();
    }

    void connect(const std::string& host, const std::string& port) {
        // look up domain name
        const auto resolve_result = resolver.resolve(host, port);

        // set timeout and connect
        std::cerr << "connecting..." << std::endl;
        beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(30));
        const auto connect_result = beast::get_lowest_layer(ws).connect(resolve_result);
        
        // From example: turn off the timeout on the tcp_stream, because
        // the websocket stream has its own timeout system.
        beast::get_lowest_layer(ws).expires_never();
        ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));

        // From example: Host HTTP header during the WebSocket handshake.
        // See https://tools.ietf.org/html/rfc7230#section-5.4
        std::cerr << "performing handshake..." << std::endl;
        const std::string host_header = host + ":" + std::to_string(connect_result.port());
        ws.handshake(host_header, "/");

        thread.reset(new std::thread(&Client::read_loop, this));
    }
};
