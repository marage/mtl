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

  explicit Connection(Terminal t) : t_(t) {
  }

  inline Terminal terminal() const { return t_; }
  inline bool IsConnected() const { return false; }
  inline bool IsBusy() const { return true; }
  inline HostAddress LocalAddress() const { return t_->localEndpoint(); }
  inline void Send(OutRequest& oreq) {
    t_->send(oreq);
  }

  inline void SendTo(OutRequest& /*request*/, const AsioUDPEndpoint& /*to*/) {}

private:
  Terminal t_;
};

template <>
class Connection<network::tcp::ConnectionPtr> {
public:
  typedef network::tcp::ConnectionPtr Terminal;

  explicit Connection(Terminal terminal) : t_(terminal) {
  }

  inline Terminal terminal() const { return t_; }
  inline bool IsConnected() const { return true; }
  inline bool IsBusy() const { return t_->IsBusy(); }
  inline HostAddress LocalAddress() const {
    return HostAddress(t_->LocalEndpoint());
  }
  inline void Send(OutRequest& oreq) {
    t_->Send(oreq);
  }

  inline void SendTo(OutRequest& /*oreq*/, const AsioUDPEndpoint& /*to*/) {}

private:
  Terminal t_;
};

template <>
class Connection<network::tcp::ClientPtr> {
public:
  typedef network::tcp::ClientPtr Terminal;

  explicit Connection(Terminal terminal) : t_(terminal) {
  }

  inline Terminal terminal() const { return t_; }
  inline bool IsConnected() const { return t_->IsConnected(); }
  inline bool IsBusy() const { return t_->IsBusy(); }
  inline HostAddress LocalAddress() const {
    return HostAddress(t_->LocalEndpoint());
  }
  inline void Send(OutRequest& oreq) {
    t_->Send(oreq);
  }
  inline void SendTo(OutRequest& /*oreq*/, const AsioUDPEndpoint& /*to*/) {}

private:
  Terminal t_;
};

template <>
class Connection<network::udp::DgramPtr> {
public:
  typedef network::udp::DgramPtr Terminal;

  explicit Connection(Terminal terminal) : t_(terminal) {
  }

  inline Terminal terminal() const { return t_; }
  inline bool IsConnected() const { return true; }
  inline bool IsBusy() const { return false; }
  inline HostAddress LocalAddress() const {
    return HostAddress(t_->LocalEndpoint());
  }
  inline void Send(OutRequest& /*oreq*/) {}
  inline void SendTo(OutRequest& oreq, const AsioUDPEndpoint& to) {
    t_->SendTo(oreq, to);
  }

private:
  Terminal t_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_CONNECTION_H
