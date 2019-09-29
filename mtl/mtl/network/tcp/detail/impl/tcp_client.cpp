#include "mtl/network/tcp/client.hpp"
#include <boost/asio.hpp>

namespace mtl {
namespace network {
namespace tcp {

Client::Client(boost::asio::io_context& context)
    : status_(kUnconnected), context_(context)
{
}

Client::~Client()
{
}

TcpEndpoint Client::localEndpoint() const
{
    TcpEndpoint endpoint;
    if (isConnected()) {
        endpoint = connection_->localEndpoint();
    }
    return endpoint;
}

bool Client::isConnected() const
{
    return (connection_ ? connection_->isConnected() : false);
}

bool Client::isBusy() const
{
    return (connection_ ? connection_->isBusy() : false);
}

int Client::pendingCount() const
{
    return (connection_ ? connection_->pendingCount() : 0);
}

std::chrono::system_clock::time_point Client::lastRequestTime() const
{
    return (connection_ ? connection_->lastRequestTime() :
                          std::chrono::system_clock::time_point{});
}

bool Client::open(const std::string& host, const std::string& port)
{
    if (status_ == kConnecting || isConnected()) {
        return false;
    }

    // Start an asynchronous resolve to translate the server and service names
    // into a list of endpoints.
    boost::asio::ip::tcp::resolver resolver(context_);
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
    connection_.reset(new Connection(context_));
    connection_->packet_arrival_signal.connect(packet_arrival_signal);
    connection_->close_signal.connect(close_signal);

    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
    boost::asio::async_connect(connection_->socket(), it,
                               boost::bind(&Client::handleConnect, shared_from_this(),
                                           boost::asio::placeholders::error));

    return true;
}

void Client::send(OutRequest& oreq)
{
    if (isConnected()) {
        connection_->send(oreq);
    }
}

void Client::close()
{
    if (connection_) {
        connection_->close();
        connection_.reset();
        status_ = kUnconnected;
    }
}

void Client::handleConnect(const boost::system::error_code& ec)
{
    if (!ec) {
        status_ = kConnected;
        new_connection_signal(ec);
        connection_->start();
    } else {
        status_ = kUnconnected;
        new_connection_signal(ec);
        connection_->close();
    }
}

} // namespace tcp
} // namespace network
} // namespace mtl
