#ifndef MTL_FRAMEWORK_CORE_SINGLETON_TCP_SERVER_H
#define MTL_FRAMEWORK_CORE_SINGLETON_TCP_SERVER_H
#include "mtl/singleton/singleton.hpp"
#include "tcp_server.h"

namespace mtl {
namespace framework {
namespace core {

template <typename ControllerFactory>
class MTL_EXPORT SingletonTcpServer
        : public Singleton<SingletonTcpServer<ControllerFactory>>
        , public TcpServer<ControllerFactory> {
public:
    typedef Singleton<SingletonTcpServer<ControllerFactory>> Base;

    explicit SingletonTcpServer(network::ContextPool& cp,
                                ControllerFactory* controller_factory)
        : TcpServer<ControllerFactory>(cp, controller_factory)
    {
    }

    static SingletonTcpServer<ControllerFactory>& getSingleton()
    {
        return *Base::ms_Singleton;
    }

    static SingletonTcpServer<ControllerFactory>* getSingletonPtr()
    {
        return Base::ms_Singleton;
    }
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_SINGLETON_TCP_SERVER_H
