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
class Connection
{
public:
    typedef T Terminal;

    explicit Connection(Terminal t) : t_(t)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return false; }
    bool isBusy() const { return true; }
    HostAddress localAddress() const { return t_->localEndpoint(); }
    void send(OutRequest& oreq)
    {
        t_->send(oreq);
    }

    void sendTo(OutRequest& /*request*/, const UDPEndpoint& /*to*/) {}

private:
    Terminal t_;
};

template <>
class Connection<network::tcp::connection_ptr>
{
public:
    typedef network::tcp::connection_ptr Terminal;

    explicit Connection(Terminal terminal)
        : t_(terminal)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return true; }
    bool isBusy() const { return t_->isBusy(); }
    HostAddress localAddress() const { return HostAddress(t_->localEndpoint()); }
    void send(OutRequest& oreq)
    {
        t_->send(oreq);
    }

    void sendTo(OutRequest& /*oreq*/, const UDPEndpoint& /*to*/) {}

private:
    Terminal t_;
};

template <>
class Connection<network::tcp::client_ptr>
{
public:
    typedef network::tcp::client_ptr Terminal;

    explicit Connection(Terminal terminal)
        : t_(terminal)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return t_->isConnected(); }
    bool isBusy() const { return t_->isBusy(); }
    HostAddress localAddress() const { return HostAddress(t_->localEndpoint()); }
    void send(OutRequest& oreq)
    {
        t_->send(oreq);
    }
    void sendTo(OutRequest& /*oreq*/, const UDPEndpoint& /*to*/) {}

private:
    Terminal t_;
};

template <>
class Connection<network::udp::dgram_ptr>
{
public:
    typedef network::udp::dgram_ptr Terminal;

    explicit Connection(Terminal terminal)
        : t_(terminal)
    {
    }

    Terminal terminal() const { return t_; }
    bool isConnected() const { return true; }
    bool isBusy() const { return false; }
    HostAddress localAddress() const { return HostAddress(t_->localEndpoint()); }
    void send(OutRequest& /*oreq*/) {}
    void sendTo(OutRequest& oreq, const UDPEndpoint& to)
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
