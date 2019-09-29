#include "mtl/task/task.hpp"

namespace mtl {

Task::Task(int type, int timeout)
    : type_(type), state_(kInactive)
{
    if (timeout > 0) {
        dead_time_ = std::chrono::system_clock::now() + std::chrono::milliseconds(timeout);
    }
}

void Task::activate()
{
    state_ = kActive;
    activateImpl();
}

Task::State Task::process()
{
    if (dead_time_.time_since_epoch() != std::chrono::system_clock::time_point::duration::zero()) {
        if (dead_time_ <= std::chrono::system_clock::now()) {
            state_ = kFailed;
            return kFailed;
        }
    }
    return ((state_ == kActive) ? processImpl() : state_);
}

} // namespace mtl
