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
    explicit HeartBeatTask(network::tcp::ClientPtr client,
                           int timeout = 5000);

protected:
    State processImpl() override;

private:
    network::tcp::ClientPtr client_;
    int timeout_;
};

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_HEART_BEAT_TASK_H
