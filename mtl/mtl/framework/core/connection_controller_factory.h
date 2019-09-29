#ifndef MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_FACTORY_H
#define MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_FACTORY_H
#include <boost/shared_ptr.hpp>
#include "mtl/network/tcp/connection.hpp"

namespace mtl {
namespace framework {
namespace core {

template <typename T>
class ConnectionControllerFactory
{
public:
    typedef T Controller;
    typedef boost::shared_ptr<T> ControllerPtr;

    ControllerPtr createController(uint64_t id,
                                   network::tcp::ConnectionPtr c) {
        return ControllerPtr(new T(id, c));
    }
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_CONNECTION_CONTROLLER_FACTORY_H
