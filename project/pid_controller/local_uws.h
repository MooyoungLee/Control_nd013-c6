#ifndef LOCAL_UWS_H
#define LOCAL_UWS_H

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

namespace uWS {

enum Mode { SERVER };

enum class OpCode {
  TEXT
};

class HttpRequest {};

namespace detail {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = boost::beast::websocket;
using tcp = asio::ip::tcp;

struct Session {
  explicit Session(tcp::socket socket) : stream(std::move(socket)) {}

  websocket::stream<tcp::socket> stream;
};

}  // namespace detail

template <Mode mode>
class WebSocket {
public:
  WebSocket() = default;

  explicit WebSocket(std::shared_ptr<detail::Session> session)
      : session_(std::move(session)) {}

  void send(const char *data, std::size_t length, OpCode op_code) {
    if (!session_) {
      return;
    }
    session_->stream.text(op_code == OpCode::TEXT);
    session_->stream.write(detail::asio::buffer(data, length));
  }

  void close() {
    if (!session_) {
      return;
    }
    boost::system::error_code ec;
    session_->stream.close(detail::websocket::close_code::normal, ec);
  }

private:
  std::shared_ptr<detail::Session> session_;
};

class Hub {
public:
  using MessageHandler =
      std::function<void(WebSocket<SERVER>, char *, std::size_t, OpCode)>;
  using ConnectionHandler =
      std::function<void(WebSocket<SERVER>, HttpRequest)>;
  using DisconnectionHandler =
      std::function<void(WebSocket<SERVER>, int, char *, std::size_t)>;

  Hub() : acceptor_(io_context_) {}

  void onMessage(MessageHandler handler) {
    on_message_ = std::move(handler);
  }

  void onConnection(ConnectionHandler handler) {
    on_connection_ = std::move(handler);
  }

  void onDisconnection(DisconnectionHandler handler) {
    on_disconnection_ = std::move(handler);
  }

  bool listen(const std::string &host, int port) {
    using detail::tcp;

    boost::system::error_code ec;
    tcp::endpoint endpoint;
    if (host == "0.0.0.0") {
      endpoint = tcp::endpoint(tcp::v4(), static_cast<unsigned short>(port));
    } else {
      const auto address = detail::asio::ip::make_address(host, ec);
      if (ec) {
        return false;
      }
      endpoint = tcp::endpoint(address, static_cast<unsigned short>(port));
    }

    acceptor_.open(endpoint.protocol(), ec);
    if (ec) {
      return false;
    }
    acceptor_.set_option(detail::asio::socket_base::reuse_address(true), ec);
    if (ec) {
      return false;
    }
    acceptor_.bind(endpoint, ec);
    if (ec) {
      return false;
    }
    acceptor_.listen(detail::asio::socket_base::max_listen_connections, ec);
    return !ec;
  }

  void run() {
    using detail::beast;
    using detail::tcp;
    using detail::websocket;

    while (true) {
      boost::system::error_code ec;
      tcp::socket socket(io_context_);
      acceptor_.accept(socket, ec);
      if (ec) {
        continue;
      }

      auto session = std::make_shared<detail::Session>(std::move(socket));
      session->stream.set_option(
          websocket::stream_base::timeout::suggested(beast::role_type::server));
      session->stream.accept(ec);
      if (ec) {
        continue;
      }

      WebSocket<SERVER> ws(session);
      if (on_connection_) {
        on_connection_(ws, HttpRequest{});
      }

      for (;;) {
        beast::flat_buffer buffer;
        session->stream.read(buffer, ec);
        if (ec) {
          NotifyDisconnect(ws, ec);
          break;
        }

        std::string payload = beast::buffers_to_string(buffer.data());
        std::vector<char> message(payload.begin(), payload.end());
        message.push_back('\0');

        if (on_message_) {
          on_message_(ws, message.data(), payload.size(), OpCode::TEXT);
        }
      }
    }
  }

private:
  void NotifyDisconnect(WebSocket<SERVER> ws, const boost::system::error_code &ec) {
    if (!on_disconnection_) {
      return;
    }
    std::string message = ec.message();
    on_disconnection_(ws, ec.value(), const_cast<char *>(message.c_str()),
                      message.size());
  }

  detail::asio::io_context io_context_;
  detail::tcp::acceptor acceptor_;
  MessageHandler on_message_;
  ConnectionHandler on_connection_;
  DisconnectionHandler on_disconnection_;
};

}  // namespace uWS

#endif  // LOCAL_UWS_H
