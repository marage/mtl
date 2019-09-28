#ifndef MTL_FRAMEWORK_CORE_LOGIC_UNIT_H
#define MTL_FRAMEWORK_CORE_LOGIC_UNIT_H
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/shared_lock_guard.hpp>
#include <boost/noncopyable.hpp>
#include "connection.h"

namespace mtl {
namespace framework {
namespace core {

template <typename T>
class LogicUnit : private boost::noncopyable {
public:
  typedef T Terminal;

  enum ProcessType {
    kOnce,
    kLoop,
  };

  LogicUnit(uint32_t id, T terminal, int process_type);
  virtual ~LogicUnit() {}

  inline uint32_t id() const { return id_; }

  inline bool active() const { return active_; }
  inline void set_active(bool active) { active_ = active; }

  void Process();
  inline void Quit() { running_ = false; }

  inline bool IsBusy() const { return connection_.IsBusy(); }
  inline HostAddress LocalAddress() const {
    return connection_.LocalAddress();
  }
  inline void Send(OutRequest& oreq) {
    connection_.Send(oreq);
  }
  inline void SendTo(OutRequest& request, const AsioUDPEndpoint& to) {
    connection_.SendTo(request, to);
  }

  inline size_t request_count() const { return requests_.size(); }
  inline void PushRequest(InRequest& request) {
    boost::shared_lock_guard<boost::shared_mutex> guard(mutex_);
    requests_.push_back(request);
  }

  inline void RegisterHandler(uint32_t cmd, const RequestHandler& handler) {
    handlers_.insert(std::make_pair(cmd, handler));
  }
  inline void AppendTask(const TaskPtr& t) {
    tasks_.Append(t);
  }

  virtual void OnConnected() {}

  static uint32_t NextSequence();

private:
  virtual void HandleRequest(InRequest& /*ireq*/) {}

  uint32_t id_;
  Connection<T> connection_;
  int process_type_;
  bool running_;
  bool active_;
  std::list<InRequest> requests_;
  boost::shared_mutex mutex_;
  RequestHandlerMap handlers_;
  TaskGroup tasks_;
};

template <typename T>
LogicUnit<T>::LogicUnit(uint32_t id, T terminal, int process_type)
  : id_(id), connection_(terminal), process_type_(process_type)
  , running_(true), active_(true), tasks_(0, true, 0) {
}

template <typename T>
void LogicUnit<T>::Process() {
  bool found;
  InRequest ireq;

  do {
    // when active
    if (active_) {
      // process network requests
      {
        boost::shared_lock_guard<boost::shared_mutex> guard(mutex_);
        found = !requests_.empty();
        if (found) {
          ireq = requests_.front();
          requests_.pop_front();
        }
      }
      if (found) {
        uint32_t cmd = ireq.readCommand();
        auto it = handlers_.find(cmd);
        if (it != handlers_.end()) {
          it->second(ireq);
        } else {
          HandleRequest(ireq);
        }
      }
      // process tasks
      if (tasks_.IsInactive() || tasks_.IsCompleted()) {
        tasks_.Activate();
      }
      tasks_.Process();
    }
  } while (running_ && process_type_ == kLoop);
}

template <typename T>
uint32_t LogicUnit<T>::NextSequence() {
  static uint32_t next = 0;
  ++next;
  next = next % 0xffffffff;
  return next;
}

typedef LogicUnit<network::tcp::ClientPtr> ClientLogicUnit;

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_LOGIC_UNIT_H
