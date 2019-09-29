//
// connection.cpp
// ~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "mtl/network/tcp/connection.hpp"
#include <boost/bind.hpp>
#include "mtl/network/singleton_buffer_pool.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/detail/byte_order.hpp"
#include "mtl/network/in_request.hpp"
#include "mtl/network/out_request.hpp"

namespace mtl {
namespace network {
namespace tcp {

Connection::Connection(boost::asio::io_context& context)
    : socket_(context), connected_(true), send_count_(0), sent_count_(0)
{
    buffer_ = SingletonBufferPool::getSingleton().allocBuffer(1024);
    last_request_time_ = std::chrono::system_clock::now();
}

TcpEndpoint Connection::localEndpoint() const
{
    boost::system::error_code ec;
    return socket_.local_endpoint(ec);
}

TcpEndpoint Connection::remoteEndpoint() const
{
    boost::system::error_code ec;
    return socket_.remote_endpoint(ec);
}

void Connection::start()
{
    socket_.async_read_some(boost::asio::buffer(buffer_.get(), 1024),
                            boost::bind(&Connection::handleRead, shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
}

void Connection::send(OutRequest& request)
{
    ++send_count_;
    last_request_time_ = std::chrono::system_clock::now();
    request.writeLength(request.size());
    boost::asio::async_write(socket_,
                             boost::asio::buffer(request.buffer().get(), request.size()),
                             boost::bind(&Connection::handleWrite, shared_from_this(),
                                         request.buffer(), boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

void Connection::close()
{
    connected_ = false;
    // disconnect signals
    packet_arrival_signal.disconnect_all_slots();
    close_signal.disconnect_all_slots();
    // close the socket
    boost::system::error_code ignored_ec;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
    socket_.close();
}

void Connection::handleRead(const boost::system::error_code& ec,
                            std::size_t bytes_transferred)
{
    if (!ec) {
        stream_.write(buffer_, bytes_transferred);
        boost::uint16_t len = 0;
        SharedBuffer buf;
        while (isIntactRequest(len)) {
            buf = SingletonBufferPool::getSingleton().allocBuffer(len);
            stream_.read(buf.get(), len);
            handlePacketArrival(buf, len);
        }
        buffer_ = SingletonBufferPool::getSingleton().allocBuffer(1024);
        socket_.async_read_some(boost::asio::buffer(buffer_.get(), 1024),
                                boost::bind(&Connection::handleRead, shared_from_this(),
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
    } else {
        connected_ = false;
        // Initiate graceful connection closure.
        boost::system::error_code ignored_ec;
        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
        close_signal(ec);
    }
    // If an error occurs then no new asynchronous operations are started. This
    // means that all shared_ptr references to the connection object will
    // disappear and the object will be destroyed automatically after this
    // handler returns. The connection class's destructor closes the socket.
}

void Connection::handleWrite(SharedBuffer /*buffer*/, const boost::system::error_code& ec,
                             std::size_t /*bytes_transferred*/)
{
    if (!ec) {
        ++sent_count_;
    } else {
        connected_ = false;
        // Initiate graceful connection closure.
        boost::system::error_code ignored_ec;
        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
        close_signal(ec);
    }

    // No new asynchronous operations are started. This means that all shared_ptr
    // references to the connection object will disappear and the object will be
    // destroyed automatically after this handler returns. The connection class's
    // destructor closes the socket.
}

void Connection::handlePacketArrival(const SharedBuffer& buffer, size_t len)
{
    InRequest ireq(buffer, static_cast<uint16_t>(len));
    packet_arrival_signal(ireq);
}

bool Connection::isIntactRequest(boost::uint16_t& req_length)
{
    size_t len = stream_.length();
    if (len >= kHeaderLength) {
        stream_.peek(&req_length, sizeof(boost::int16_t));
        req_length = NetworkToHost16(req_length);
        return (static_cast<size_t>(req_length) <= len);
    }
    return false;
}

} // namespace tcp
} // namespace network
} // namespace mtl
