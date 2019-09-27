#ifndef MTL_FRAMEWORK_CORE_TCP_SERVER_H
#define MTL_FRAMEWORK_CORE_TCP_SERVER_H
#include <boost/thread.hpp>
#include "mtl/network/context_pool.hpp"
#include "mtl/network/tcp/server.hpp"
#include "tcp_server_work.h"

namespace mtl {
namespace framework {
namespace core {

template <typename ControllerFactory>
class TCPServer : private boost::noncopyable
{
    typedef TCPServerWork<ControllerFactory> Work;

public:
    typedef TCPServer<ControllerFactory> This;
    typedef typename ControllerFactory::Controller Controller;
    typedef typename ControllerFactory::controller_ptr controller_ptr;

    explicit TCPServer(network::ContextPool& cp,
                       ControllerFactory* controller_factory);
    ~TCPServer() {}

    bool open(const std::string& address, unsigned short port, size_t work_count = 1);
    bool close();

    boost::asio::ip::tcp::endpoint endpoint() const
    {
        return server_->localEndpoint();
    }

    bool containsController(uint32_t id) const;
    size_t controllerCount() const;
    controller_ptr controller(uint32_t id) const;
    controller_ptr controller(const TCPEndpoint& ep) const;
    controller_ptr getIdleController(uint16_t main_cmd);
    void getMainCommands(std::set<uint16_t>& cmds);

private:
    bool handleNewConnection(network::tcp::connection_ptr connection);

    uint32_t nextId();

    network::tcp::server_ptr server_;
    ControllerFactory* controller_factory_;
    std::vector<boost::thread*> threads_;
    std::vector<Work*> works_;
};

template <typename ControllerFactory>
TCPServer<ControllerFactory>::TCPServer(network::ContextPool& cp,
                                        ControllerFactory* controller_factory)
    : server_(new network::tcp::Server(cp)), controller_factory_(controller_factory)
{
    server_->new_connection_signal.connect(boost::bind(&TCPServer::handleNewConnection, this, _1));
}

template <typename ControllerFactory>
bool TCPServer<ControllerFactory>::open(const std::string& address, unsigned short port,
                                        size_t work_count)
{
    if (!server_->open(address, port)) {
        return false;
    }

    ControllerIdGenerator id_generator = std::bind(&This::nextId, this);

    // create work threads
    threads_.reserve(work_count);
    works_.reserve(work_count);
    for (size_t i = 0; i < work_count; ++i) {
        Work* w = new Work(controller_factory_, id_generator);
        works_.push_back(w);
        boost::thread* t = new boost::thread(&Work::process, w);
        threads_.push_back(t);
    }

    return true;
}

template <typename ControllerFactory>
bool TCPServer<ControllerFactory>::close()
{
    bool ret = server_->close();

    // stop works
    for (typename std::vector<Work*>::iterator it = works_.begin(), end = works_.end();
         it != end; ++it) {
        (*it)->quit();
    }
    // stop and delete threads
    for (typename std::vector<boost::thread*>::iterator it = threads_.begin(), end = threads_.end();
         it != end; ++it) {
        if ((*it)->joinable())
            (*it)->join();
        delete *it;
    }
    threads_.clear();
    // delete works
    for (typename std::vector<Work*>::iterator it = works_.begin(), end = works_.end();
         it != end; ++it) {
        delete *it;
    }
    works_.clear();

    return ret;
}

template <typename ControllerFactory>
bool TCPServer<ControllerFactory>::containsController(uint32_t id) const
{
    for (const auto& w: works_) {
        if (w->containsController(id)) {
            return true;
        }
    }
    return false;
}

template <typename ControllerFactory>
size_t TCPServer<ControllerFactory>::controllerCount() const
{
    size_t count = 0;
    for (const auto& w: works_) {
        count += w->controllerCount();
    }
    return count;
}

template <typename ControllerFactory>
typename TCPServer<ControllerFactory>::controller_ptr TCPServer<ControllerFactory>::controller(uint32_t id) const
{
    controller_ptr c;
    for (const auto& w: works_) {
        c = w->controller(id);
        if (c) {
            break;
        }
    }
    return c;
}

template <typename ControllerFactory>
typename TCPServer<ControllerFactory>::controller_ptr TCPServer<ControllerFactory>::controller(const TCPEndpoint& ep) const
{
    controller_ptr c;
    for (const auto& w: works_) {
        c = w->controller(ep);
        if (c) {
            break;
        }
    }
    return c;
}

template <typename ControllerFactory>
typename TCPServer<ControllerFactory>::controller_ptr TCPServer<ControllerFactory>::getIdleController(
        uint16_t main_cmd)
{
    controller_ptr c;
    for (const auto& w: works_) {
        c = w->getIdleController(main_cmd);
        if (c) {
            break;
        }
    }
    return c;
}

template <typename ControllerFactory>
void TCPServer<ControllerFactory>::getMainCommands(std::set<uint16_t>& cmds)
{
    for (const auto& w: works_) {
        w->getMainCommands(cmds);
    }
}

template <typename ControllerFactory>
bool TCPServer<ControllerFactory>::handleNewConnection(network::tcp::connection_ptr connection)
{
    size_t min_count = 0xffffffff;
    Work* w;
    for (typename std::vector<Work*>::iterator it = works_.begin(), end = works_.end();
         it != end; ++it) {
        if ((*it)->controllerCount() < min_count) {
            w = *it;
            min_count = (*it)->controllerCount();
        }
    }
    return w->handleNewConnection(connection);
}

template <typename ControllerFactory>
uint32_t TCPServer<ControllerFactory>::nextId()
{
    static uint32_t id = 0;
    ++id;
    do {
        id = id % 0xffffffff;
    } while (containsController(id));
    return id;
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_TCP_SERVER_H
