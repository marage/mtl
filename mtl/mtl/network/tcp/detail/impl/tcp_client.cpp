#include "mtl/network/tcp/client.hpp"
#include <boost/asio.hpp>
#include "mtl/network/in_request.hpp"

namespace mtl {
namespace network {
namespace tcp {

Client::Client(boost::asio::io_context& io_context)
    : status_(UNCONNECTED), io_context_(io_context)
{
}

Client::~Client()
{
}

boost::asio::ip::tcp::endpoint Client::localEndpoint() const
{
    boost::asio::ip::tcp::endpoint endpoint;
    if (isConnected()) {
        endpoint = connection_->localEndpoint();
    }
    return endpoint;
}

bool Client::isConnected() const
{
    return status_ == CONNECTED;
}

bool Client::isBusy() const
{
    return connection_->isBusy();
}

int Client::pendingCount() const
{
    return connection_->pendingCount();
}

std::chrono::system_clock::time_point Client::lastRequestTime() const
{
    return connection_->lastRequestTime();
}

bool Client::open(const std::string& server, const std::string& port)
{
    if (status_ != UNCONNECTED)
        return false;

    // Start an asynchronous resolve to translate the server and service names
    // into a list of endpoints.
    boost::asio::ip::tcp::resolver resolver(io_context_);
    boost::asio::ip::tcp::resolver::query query(server, port);
    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver::iterator it = resolver.resolve(query, ec);
    if (it == boost::asio::ip::tcp::resolver::iterator()) {
        return false;
    }

    // New a connection
    connection_.reset(new Connection(io_context_));
    connection_->packet_arrival_signal.connect(packet_arrival_signal);
    connection_->close_signal.connect(close_signal);

    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
    boost::asio::async_connect(connection_->socket(), it,
                               boost::bind(&Client::handleConnect, shared_from_this(),
                                           boost::asio::placeholders::error));

    status_ = CONNECTING;

    return true;
}

void Client::send(OutRequest& oreq)
{
    if (isConnected())
        connection_->send(oreq);
}

void Client::close()
{
    if (isConnected()) {
        connection_->close();
        status_ = UNCONNECTED;
    }
    connection_.reset();
}

void Client::handleConnect(const boost::system::error_code& ec)
{
    if (!ec) {
        status_ = CONNECTED;
        new_connection_signal(ec);
        connection_->start();
    } else {
        status_ = UNCONNECTED;
        new_connection_signal(ec);
        connection_->close();
    }
}

} // namespace tcp
} // namespace network
} // namespace mtl
