#ifndef MTL_FRAMEWORK_CORE_SINGLETON_TCP_SERVER_H
#define MTL_FRAMEWORK_CORE_SINGLETON_TCP_SERVER_H
#include "mtl/singleton/singleton.hpp"
#include "tcp_server.h"

namespace mtl {
namespace framework {
namespace core {

template <typename ControllerFactory>
class MTL_EXPORT SingletonTCPServer
    : public Singleton<SingletonTCPServer<ControllerFactory>>
    , public TCPServer<ControllerFactory> {
public:
  typedef Singleton<SingletonTCPServer<ControllerFactory>> Base;

  explicit SingletonTCPServer(network::IOServicePool& isp,
                              ControllerFactory* controller_factory)
    : TCPServer<ControllerFactory>(isp, controller_factory) {
  }

  static SingletonTCPServer<ControllerFactory>& getSingleton() {
    return *Base::ms_Singleton;
  }

  static SingletonTCPServer<ControllerFactory>* getSingletonPtr() {
    return Base::ms_Singleton;
  }
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_SINGLETON_TCP_SERVER_H
