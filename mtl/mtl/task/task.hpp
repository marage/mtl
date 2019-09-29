#ifndef MTL_TASK_HPP
#define MTL_TASK_HPP
#include <memory>
#include "mtl/mtl.hpp"

namespace mtl {

class MTL_EXPORT Task
{
public:
    enum State
    {
        kInactive,
        kActive,
        kCompleted,
        kFailed
    };

    explicit Task(int type, int timeout = 10000);
    virtual ~Task() {}

    int type() const { return type_; }
    bool isInactive()const { return state_ == kInactive; }
    bool isActive()const { return state_ == kActive; }
    bool isCompleted()const { return state_ == kCompleted; }
    bool isFailed()const { return state_ == kFailed; }
    State state() const { return state_; }
    const std::chrono::system_clock::time_point& deadTime() const
    {
        return dead_time_;
    }
    bool isTimeout(const std::chrono::system_clock::time_point& now) const
    {
        return (now >= dead_time_);
    }

    void activate();
    State process();

protected:
    void setState(State s) { state_ = s; }

private:
    virtual void activateImpl() {}
    virtual State processImpl() = 0;

    int type_;
    State state_;
    std::chrono::system_clock::time_point dead_time_;
};

typedef std::shared_ptr<Task> TaskPtr;

} // namespace mtl

#endif // MTL_TASK_HPP
