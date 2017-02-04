#ifndef MTL_NETWORK_SHARED_BUFFER_HPP
#define MTL_NETWORK_SHARED_BUFFER_HPP
#include <boost/smart_ptr/shared_ptr.hpp>

namespace mtl {
namespace network {

typedef boost::shared_ptr<char> SharedBuffer;

} // network
} // mtl

#endif // MTL_NETWORK_SHARED_BUFFER_HPP
