//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "mtl/network/tcp/server.hpp"
#include <boost/bind.hpp>

namespace mtl {
namespace network {
namespace tcp {

Server::Server(IOServicePool& isp)
    : io_service_pool_(isp), acceptor_(io_service_pool_.getIOService())
{
}

Server::~Server()
{
}

bool Server::open(const std::string& address, unsigned short port)
{
    // Open the acceptor with the option to reuse the address (i.e. SO_REUSEADDR).
    boost::asio::ip::address addr = boost::asio::ip::address::from_string(address);
    boost::asio::ip::tcp::endpoint endpoint(addr, port);
    acceptor_.open(boost::asio::ip::tcp::v4());
    acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    startAccept();
    return true;
}

bool Server::close()
{
    boost::system::error_code ec;
    acceptor_.close(ec);
    return !ec;
}

boost::asio::ip::tcp::endpoint Server::localEndpoint() const
{
    boost::system::error_code ec;
    return acceptor_.local_endpoint(ec);
}

void Server::startAccept()
{
    new_connection_.reset(new Connection(
                              io_service_pool_.getIOService()));
    acceptor_.async_accept(new_connection_->socket(),
                           boost::bind(&Server::handleAccept, this,
                                       boost::asio::placeholders::error));
}

void Server::handleAccept(const boost::system::error_code& e)
{
    if (!e) {
        if (new_connection_signal(new_connection_))
            new_connection_->start();
    }

    startAccept();
}
} // namespace tcp
} // namespace network
} // namespace mtl
