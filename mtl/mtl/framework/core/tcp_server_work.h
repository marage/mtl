#ifndef MTL_FRAMEWORK_CORE_TCP_SERVER_WORK_H
#define MTL_FRAMEWORK_CORE_TCP_SERVER_WORK_H
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <iostream>
#include <time.h>
#include <boost/noncopyable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "mtl/utility/util.hpp"
#include "mtl/framework/log/log.h"
#include "mtl/network/tcp/connection.hpp"
#include "framework.h"

namespace mtl {
namespace framework {
namespace core {

typedef std::function<uint32_t()> ControllerIdGenerator;

template <typename ControllerFactory>
class TcpServerWork : private boost::noncopyable
{
public:
    typedef typename ControllerFactory::ControllerPtr ControllerPtr;

    TcpServerWork(ControllerFactory* f, const ControllerIdGenerator& g)
        : controller_factory_(f), id_generator_(g), running_(true)
    {
    }

    size_t controllerCount() const { return controllers_.size(); }
    bool containsController(uint32_t id)
    {
        bool found;
        mutex_.lock();
        found = (controllers_.find(id) != controllers_.end());
        mutex_.unlock();
        return found;
    }
    ControllerPtr getController(uint32_t id)
    {
        ControllerPtr p;
        mutex_.lock();
        typename ControllerMap::const_iterator it = controllers_.find(id);
        if (it != controllers_.end()) {
            p = it->second;
        }
        mutex_.unlock();
        return p;
    }
    ControllerPtr getController(const network::TcpEndpoint& ep)
    {
        ControllerPtr p;
        mutex_.lock();
        typename ControllerMap::const_iterator it = controllers_.begin();
        typename ControllerMap::const_iterator end = controllers_.end();
        for (; it != end; ++it) {
            if (it.second->remote_endpoint() == ep) {
                break;
            }
        }
        if (it != end) {
            p = it.second;
        }
        mutex_.unlock();
        return p;
    }
    ControllerPtr getIdleController(uint16_t main_cmd)
    {
        ControllerPtr p;
        mutex_.lock();
        uint64_t count = 0xffffffffffffffff;
        typename ControllerMap::iterator it = controllers_.begin();
        typename ControllerMap::iterator end = controllers_.end();
        typename ControllerMap::iterator best_it = end;
        for (; it != end; ++it) {
            if (it->second->mainCommand() == main_cmd &&
                it->second->accessCount() < count) {
                best_it = it;
                count = it->second->accessCount();
            }
        }
        if (best_it != end) {
            p = best_it->second;
            best_it->second->setAccessCount(++count);
        }
        mutex_.unlock();
        return p;
    }
    void getMainCommands(std::set<uint16_t>& cmds)
    {
        mutex_.lock();
        typename ControllerMap::iterator it = controllers_.begin();
        typename ControllerMap::iterator end = controllers_.end();
        for (; it != end; ++it) {
            cmds.insert(it->second->mainCommand());
        }
        mutex_.unlock();
    }

    void process();

    bool handleNewConnection(network::tcp::ConnectionPtr c)
    {
        ControllerPtr cp(controller_factory_->createController(id_generator_(), c));
        new_mutex_.lock();
        new_controllers_.push_back(cp);
        new_mutex_.unlock();
        return true;
    }

    void quit() { running_ = false; }

private:
    typedef typename std::unordered_map<uint32_t, ControllerPtr> ControllerMap;

    ControllerFactory* controller_factory_;
    ControllerIdGenerator id_generator_;
    bool running_;
    typename std::list<ControllerPtr> new_controllers_;
    boost::mutex new_mutex_;
    ControllerMap controllers_;
    boost::mutex mutex_;
};

template <typename ControllerFactory>
void TcpServerWork<ControllerFactory>::process()
{
    typedef std::chrono::duration<int64_t, std::milli> MilliType;
    std::chrono::system_clock::time_point now;
    std::chrono::system_clock::duration d;

    while (running_) {
        now = std::chrono::system_clock::now();
        // push all new clients to active client list
        new_mutex_.lock();
        {
            mutex_.lock();
            for (typename std::list<ControllerPtr>::iterator it = new_controllers_.begin(),
                 end = new_controllers_.end(); it != end; ++it) {
                controllers_.insert(std::make_pair((*it)->id(), *it));
            }
            mutex_.unlock();
        }
        new_controllers_.clear();
        new_mutex_.unlock();
        // process all connection requests
        for (typename ControllerMap::iterator it = controllers_.begin(),
             end = controllers_.end(); it != end;) {
            d = now - it->second->when();
            if ((std::chrono::duration_cast<MilliType>(d)).count() >
                (it->second->active() ? 15000 : 2000)) {
                it->second->close();
                mutex_.lock();
                it = controllers_.erase(it);
                mutex_.unlock();
            } else {
                it->second->process();
                ++it;
            }
        }
    }
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_TCP_SERVER_WORK_H
