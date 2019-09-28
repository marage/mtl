#ifndef MTL_TASK_HPP
#define MTL_TASK_HPP
#include <memory>
#include "mtl/mtl.hpp"

namespace mtl {

class MTL_EXPORT Task {
public:
  enum State {
    kInactive,
    kActive,
    kCompleted,
    kFailed
  };

  explicit Task(int type, int timeout = 10000);
  virtual ~Task() {}

  inline int type() const { return type_; }
  inline bool IsInactive()const { return state_ == kInactive; }
  inline bool IsActive()const { return state_ == kActive; }
  inline bool IsCompleted()const { return state_ == kCompleted; }
  inline bool IsFailed()const { return state_ == kFailed; }
  inline State state() const { return state_; }
  inline const TimePoint& dead_time() const {
    return dead_time_;
  }
  inline bool IsTimeout(const TimePoint& now) const {
    return (now >= dead_time_);
  }

  void Activate();
  State Process();

protected:
  inline void set_state(State s) { state_ = s; }

private:
  virtual void ActivateImpl() {}
  virtual State ProcessImpl() = 0;

  int type_;
  State state_;
  TimePoint dead_time_;
};

typedef std::shared_ptr<Task> TaskPtr;

} // namespace mtl

#endif // MTL_TASK_HPP
