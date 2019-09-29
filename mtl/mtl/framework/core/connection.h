#ifndef MTL_FRAMEWORK_CORE_CONNECTION_H
#define MTL_FRAMEWORK_CORE_CONNECTION_H
#include "mtl/network/tcp/connection.hpp"
#include "mtl/network/tcp/client.hpp"
#include "mtl/network/udp/dgram.hpp"
#include "framework.h"
#include "host_address.h"

namespace mtl {
namespace framework {
namespace core {

template <typename T>
class Connection {
public:
    typedef T Terminal;

    explicit Connection(Terminal t) : t_(t)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return false; }
    bool isBusy() const { return true; }
    HostAddress localAddress() const { return t_->localEndpoint(); }
    void send(network::OutRequest& oreq)
    {
        t_->send(oreq);
    }

    void sendTo(network::OutRequest& /*request*/, const network::UdpEndpoint& /*to*/) {}

private:
    Terminal t_;
};

template <>
class Connection<network::tcp::ConnectionPtr>
{
public:
    typedef network::tcp::ConnectionPtr Terminal;

    explicit Connection(Terminal terminal) : t_(terminal)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return true; }
    bool isBusy() const { return t_->isBusy(); }
    HostAddress localAddress() const
    {
        return HostAddress(t_->localEndpoint());
    }
    void send(network::OutRequest& oreq)
    {
        t_->send(oreq);
    }

    void sendTo(network::OutRequest& /*oreq*/, const network::UdpEndpoint& /*to*/) {}

private:
    Terminal t_;
};

template <>
class Connection<network::tcp::ClientPtr>
{
public:
    typedef network::tcp::ClientPtr Terminal;

    explicit Connection(Terminal terminal) : t_(terminal)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return t_->isConnected(); }
    bool isBusy() const { return t_->isBusy(); }
    HostAddress localAddress() const
    {
        return HostAddress(t_->localEndpoint());
    }
    void send(network::OutRequest& oreq)
    {
        t_->send(oreq);
    }
    void sendTo(network::OutRequest& /*oreq*/, const network::UdpEndpoint& /*to*/) {}

private:
    Terminal t_;
};

template <>
class Connection<network::udp::DgramPtr>
{
public:
    typedef network::udp::DgramPtr Terminal;

    explicit Connection(Terminal terminal) : t_(terminal)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return true; }
    bool isBusy() const { return false; }
    HostAddress localAddress() const
    {
        return HostAddress(t_->localEndpoint());
    }
    void send(network::OutRequest& /*oreq*/) {}
    void sendTo(network::OutRequest& oreq, const network::UdpEndpoint& to)
    {
        t_->sendTo(oreq, to);
    }

private:
    Terminal t_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_CONNECTION_H
