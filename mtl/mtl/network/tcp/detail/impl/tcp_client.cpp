#include "mtl/network/tcp/client.hpp"
#include <boost/asio.hpp>

namespace mtl {
namespace network {
namespace tcp {

Client::Client(boost::asio::io_service& io_service)
  : status_(kUnconnected), io_service_(io_service) {
}

Client::~Client() {
}

TCPEndpoint Client::LocalEndpoint() const {
  TCPEndpoint endpoint;
  if (IsConnected()) {
    endpoint = connection_->LocalEndpoint();
  }
  return endpoint;
}

bool Client::IsConnected() const {
  return (connection_ ? connection_->IsConnected() : false);
}

bool Client::IsBusy() const {
  return (connection_ ? connection_->IsBusy() : false);
}

int Client::PendingCount() const {
  return (connection_ ? connection_->PendingCount() : 0);
}

std::chrono::system_clock::time_point Client::LastRequestTime() const {
  return (connection_ ? connection_->last_request_time() :
                        std::chrono::system_clock::time_point{});
}

bool Client::Open(const std::string& host, const std::string& port) {
  if (status_ == kConnecting || IsConnected()) {
    return false;
  }

  // Start an asynchronous resolve to translate the server and service names
  // into a list of endpoints.
  boost::asio::ip::tcp::resolver resolver(io_service_);
  boost::asio::ip::tcp::resolver::query query(host, port);
  boost::system::error_code ec;
  auto it = resolver.resolve(query, ec);
  if (it == boost::asio::ip::tcp::resolver::iterator()) {
    return false;
  }

  // store server address
  server_host_ = host;
  server_port_ = port;
  status_ = kConnecting;

  // New a connection
  connection_.reset(new Connection(io_service_));
  connection_->packet_arrival_signal.connect(packet_arrival_signal);
  connection_->close_signal.connect(close_signal);

  // Attempt a connection to each endpoint in the list until we
  // successfully establish a connection.
  boost::asio::async_connect(connection_->socket(), it,
                             boost::bind(&Client::HandleConnect, shared_from_this(),
                                         boost::asio::placeholders::error));

  return true;
}

void Client::Send(OutRequest& oreq) {
  if (IsConnected()) {
    connection_->Send(oreq);
  }
}

void Client::Close() {
  if (connection_) {
    connection_->Close();
    connection_.reset();
    status_ = kUnconnected;
  }
}

void Client::HandleConnect(const boost::system::error_code& ec) {
  if (!ec) {
    status_ = kConnected;
    new_connection_signal(ec);
    connection_->Start();
  } else {
    status_ = kUnconnected;
    new_connection_signal(ec);
    connection_->Close();
  }
}

} // namespace tcp
} // namespace network
} // namespace mtl
