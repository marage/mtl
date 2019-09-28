#ifndef MTL_FRAMEWORK_CORE_CONNECT_TASK_H
#define MTL_FRAMEWORK_CORE_CONNECT_TASK_H
#include "mtl/task/task.hpp"
#include "mtl/network/tcp/client.hpp"
#include "framework.h"

namespace mtl {
namespace framework {
namespace core {

class ConnectTask : public Task {
public:
  explicit ConnectTask(network::tcp::ClientPtr client,
                       int timeout = 10000);

protected:
  State ProcessImpl() override;

private:
  network::tcp::ClientPtr client_;
  std::chrono::system_clock::time_point previous_;
  int timeout_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_CONNECT_TASK_H
