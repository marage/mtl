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
class LogicUnit : private boost::noncopyable
{
public:
    typedef T Terminal;

    enum ProcessType
    {
        kOnce,
        kLoop,
    };

    LogicUnit(uint32_t id, T terminal, int process_type);
    virtual ~LogicUnit() {}

    uint32_t id() const { return id_; }

    bool active() const { return active_; }
    void setActive(bool active) { active_ = active; }

    void process();
    void quit() { running_ = false; }

    bool isBusy() const { return connection_.isBusy(); }
    HostAddress localAddress() const { return connection_.localAddress(); }
    void send(network::OutRequest& oreq) { connection_.send(oreq); }
    void sendTo(network::OutRequest& request, const network::UdpEndpoint& to)
    {
        connection_.sendTo(request, to);
    }

    size_t requestCount() const { return requests_.size(); }
    void pushRequest(network::InRequest& request)
    {
        boost::shared_lock_guard<boost::shared_mutex> guard(mutex_);
        requests_.push_back(request);
    }

    void registerHandler(uint32_t cmd, const RequestHandler& handler)
    {
        handlers_.insert(std::make_pair(cmd, handler));
    }
    void appendTask(const TaskPtr& t)
    {
        tasks_.append(t);
    }

    virtual void onConnected() {}

    static uint32_t nextSequence();

private:
    virtual void handleRequest(network::InRequest& /*ireq*/) {}

    uint32_t id_;
    Connection<T> connection_;
    int process_type_;
    bool running_;
    bool active_;
    std::list<network::InRequest> requests_;
    boost::shared_mutex mutex_;
    RequestHandlerMap handlers_;
    TaskGroup tasks_;
};

template <typename T>
LogicUnit<T>::LogicUnit(uint32_t id, T terminal, int process_type)
    : id_(id), connection_(terminal), process_type_(process_type)
    , running_(true), active_(true), tasks_(0, true, 0)
{
}

template <typename T>
void LogicUnit<T>::process()
{
    bool found;
    network::InRequest ireq;

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
                    handleRequest(ireq);
                }
            }
            // process tasks
            if (tasks_.isInactive() || tasks_.isCompleted()) {
                tasks_.activate();
            }
            tasks_.process();
        }
    } while (running_ && process_type_ == kLoop);
}

template <typename T>
uint32_t LogicUnit<T>::nextSequence()
{
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
