#ifndef MTL_FRAMEWORK_H
#define MTL_FRAMEWORK_H
#include <vector>
#include <map>
#include <list>
#include <queue>
#include <set>
#include <string>
#include <functional>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "mtl/task/task_group.hpp"
#include "mtl/datetime/date_time.hpp"
#include "mtl/network/out_request.hpp"
#include "mtl/network/in_request.hpp"

#ifdef _MSC_VER
#include <Ws2tcpip.h>
#pragma warning(push)
#pragma warning(disable: 4996)
#endif

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#define GET_MAIN_COMMAND(req) ((uint16_t)(((uint32_t)(req)) >> 16))
#define GET_SUB_COMMAND(req) (((uint16_t)(req)) & 0x7FFF)
#define GET_REQUEST_TYPE(main_cmd, sub_cmd) \
  ((uint32_t)((uint16_t)(main_cmd) << 16) | (uint16_t)(sub_cmd))

namespace mtl {
namespace framework {
namespace core {

using InRequest = mtl::network::InRequest;
using OutRequest = mtl::network::OutRequest;
using DateTime = mtl::DateTime;
using RequestHandler = std::function<void(InRequest&)>;
using RequestHandlerMap = std::map<uint32_t, RequestHandler>;
using AsioUDPEndpoint = mtl::network::UdpEndpoint;
using TCPEndpoint = mtl::network::TcpEndpoint;

} // core
} // framework
} // mtl

#endif //MTL_FRAMEWORK_H
