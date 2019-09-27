#ifndef MTL_FRAMEWORK_CORE_HOST_ADDRESS_H
#define MTL_FRAMEWORK_CORE_HOST_ADDRESS_H
#include "framework.h"
#include <sstream>

namespace mtl {
namespace framework {
namespace core {

struct HostAddress
{
  std::string ip;
  uint16_t port;

  HostAddress() : port(0) {}
  explicit HostAddress(const UDPEndpoint& ep)
  {
    ip = ep.address().to_string();
    port = ep.port();
  }
  explicit HostAddress(const TCPEndpoint& ep)
  {
    ip = ep.address().to_string();
    port = ep.port();
  }
  HostAddress(const std::string& ip_, uint16_t port_)
    : ip(ip_), port(port_)
  {
  }

  HostAddress& operator=(const TCPEndpoint& ep)
  {
    ip = ep.address().to_string();
    port = ep.port();
    return *this;
  }

  HostAddress& operator=(const UDPEndpoint& ep)
  {
    ip = ep.address().to_string();
    port = ep.port();
    return *this;
  }

  bool isValid() const { return !ip.empty() && port > 0; }

  operator UDPEndpoint() const
  {
    if (ip.empty()) {
      return UDPEndpoint(boost::asio::ip::address_v4::any(), port);
    } else {
      boost::asio::ip::address addr = boost::asio::ip::address::from_string(ip);
      return UDPEndpoint(addr, port);
    }
  }

  operator TCPEndpoint() const
  {
    if (ip.empty()) {
      return TCPEndpoint(boost::asio::ip::address_v4::any(), port);
    } else {
      boost::asio::ip::address addr = boost::asio::ip::address::from_string(ip);
      return TCPEndpoint(addr, port);
    }
  }

  std::string portString() const
  {
    std::stringstream ss;
    ss << port;
    return ss.str();
  }

  static HostAddress fromString(const std::string& s)
  {
    HostAddress addr;
    std::string::size_type pos = s.find_first_of(':');
    if (pos != s.npos) {
      addr.ip = s.substr(0, pos);
      addr.port = static_cast<uint16_t>(atoi(s.substr(++pos).c_str()));
    }
    return addr;
  }

  static std::string toString(const HostAddress& addr)
  {
    std::string s = addr.ip;
    std::stringstream ss;
    ss << addr.port;
    s += ss.str();
    return s;
  }
};

inline bool operator==(const HostAddress& a, const HostAddress& b)
{
  return a.port == b.port && strcmp(a.ip.c_str(), b.ip.c_str()) == 0;
}

inline bool operator!=(const HostAddress& a, const HostAddress& b)
{
  return a.port != b.port || strcmp(a.ip.c_str(), b.ip.c_str()) != 0;
}

inline InRequest& operator>>(InRequest& ireq, HostAddress& addr)
{
  addr.ip = ireq.readString(63);
  addr.port = ireq.readInt16();
  return ireq;
}

inline OutRequest& operator<<(OutRequest& oreq, const HostAddress& addr)
{
  oreq.writeString(addr.ip);
  oreq.writeInt16(addr.port);
  return oreq;
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_HOST_ADDRESS_H
