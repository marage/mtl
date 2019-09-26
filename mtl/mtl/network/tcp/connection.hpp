//
// connection.hpp
// ~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MTL_NETWORK_CONNECTION_HPP
#define MTL_NETWORK_CONNECTION_HPP
#include <chrono>
#include <boost/asio/ip/tcp.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/signals2/signal.hpp>
#include "mtl/mtl.hpp"
#include "mtl/network/detail/byte_stream.hpp"

namespace mtl {
namespace network {

/// forward classes
class InRequest;
class OutRequest;

namespace tcp {

/// Represents a single connection from a client.
class MTL_EXPORT Connection
    : public boost::enable_shared_from_this<Connection>, private boost::noncopyable
{
public:
    /// signals
    boost::signals2::signal<void (InRequest&)> packet_arrival_signal;
    boost::signals2::signal<void (const boost::system::error_code&)> close_signal;

    /// Construct a connection with the given io_service.
    explicit Connection(boost::asio::io_context& io_context);

    /// Get the socket address
    boost::asio::ip::tcp::endpoint localEndpoint() const;
    boost::asio::ip::tcp::endpoint remoteEndpoint() const;

    /// Get the socket associated with the connection.
    boost::asio::ip::tcp::socket& socket() { return socket_; }

    /// Return whether the connection is busy.
    bool isBusy() const { return (send_count_ > (sent_count_ + 5)); }
    int pendingCount() const { return (send_count_ - sent_count_); }

    /// Return the last request time
    std::chrono::system_clock::time_point lastRequestTime() const
    {
        return last_request_time_;
    }

    /// Start the first asynchronous operation for the connection.
    void start();

    /// Aync send
    void send(OutRequest& request);

    /// close
    void close();

private:
    /// Handle completion of a read operation.
    void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred);

    /// Handle completion of a write operation.
    void handleWrite(SharedBuffer buffer, const boost::system::error_code& ec,
                     std::size_t bytes_transferred);

    /// Handle packet arrival
    void handlePacketArrival(const SharedBuffer& buffer, size_t len);

    /// Is an intact request packet.
    bool isIntactRequest(boost::int16_t& req_length);

    /// Socket for the connection.
    boost::asio::ip::tcp::socket socket_;

    /// Buffer for incoming data.
    SharedBuffer buffer_;

    /// Received byte stream
    ByteStream stream_;

    /// Stat the send & sent count
    int send_count_;
    int sent_count_;
    std::chrono::system_clock::time_point last_request_time_;
};

typedef boost::shared_ptr<Connection> connection_ptr;

} // namespace tcp
} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_CONNECTION_HPP
