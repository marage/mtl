#ifndef MTL_FRAMEWORK_CORE_HEART_BEAT_TASK_H
#define MTL_FRAMEWORK_CORE_HEART_BEAT_TASK_H
#include "mtl/task/task.hpp"
#include "mtl/network/tcp/client.hpp"
#include "framework.h"

namespace mtl {
namespace framework {
namespace core {

class HeartBeatTask final : public Task
{
public:
    explicit HeartBeatTask(network::tcp::client_ptr client,
                           int timeout = HEARTBEAT_TIMEOUT);

private:
    Status processImpl();

    network::tcp::client_ptr client_;
    int timeout_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_HEART_BEAT_TASK_H
