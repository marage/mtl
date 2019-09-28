#ifndef MTL_FRAMEWORK_CORE_TCP_SERVER_H
#define MTL_FRAMEWORK_CORE_TCP_SERVER_H
#include <boost/thread.hpp>
#include "mtl/network/io_service_pool.hpp"
#include "mtl/network/tcp/server.hpp"
#include "tcp_server_work.h"

namespace mtl {
namespace framework {
namespace core {

template <typename ControllerFactory>
class TCPServer : private boost::noncopyable {
  typedef TCPServerWork<ControllerFactory> Work;

public:
  typedef TCPServer<ControllerFactory> This;
  typedef typename ControllerFactory::Controller Controller;
  typedef typename ControllerFactory::ControllerPtr ControllerPtr;

  explicit TCPServer(network::IOServicePool& isp,
                     ControllerFactory* controller_factory);
  ~TCPServer() {}

  bool Open(const std::string& address, unsigned short port,
            size_t work_count = 1);
  bool Close();

  inline TCPEndpoint LocalEndpoint() const {
    return server_->LocalEndpoint();
  }

  bool ContainsController(uint32_t id) const {
    for (const auto& w: works_) {
      if (w->ContainsController(id))
        return true;
    }
    return false;
  }
  size_t ControllerCount() const {
    size_t count = 0;
    for (const auto& w: works_) {
      count += w->ControllerCount();
    }
    return count;
  }
  ControllerPtr GetController(uint32_t id) const {
    ControllerPtr c;
    for (const auto& w: works_) {
      c = w->GetController(id);
      if (c)
        break;
    }
    return c;
  }
  ControllerPtr GetController(const TCPEndpoint& ep) const {
    ControllerPtr c;
    for (const auto& w: works_) {
      c = w->GetController(ep);
      if (c) {
        break;
      }
    }
    return c;
  }
  ControllerPtr GetIdleController(uint16_t main_cmd) {
    ControllerPtr c;
    for (const auto& w: works_) {
      c = w->GetIdleController(main_cmd);
      if (c) {
        break;
      }
    }
    return c;
  }
  void GetMainCommands(std::set<uint16_t>& cmds) {
    for (const auto& w: works_) {
      w->GetMainCommands(cmds);
    }
  }

private:
  bool HandleNewConnection(network::tcp::ConnectionPtr connection) {
    size_t min_count = 0xffffffff;
    Work* w;
    for (typename std::vector<Work*>::iterator it = works_.begin(),
         end = works_.end(); it != end; ++it) {
      if ((*it)->controller_count() < min_count) {
        w = *it;
        min_count = (*it)->controller_count();
      }
    }
    return w->HandleNewConnection(connection);
  }

  uint32_t NextId();

  network::tcp::ServerPtr server_;
  ControllerFactory* controller_factory_;
  std::vector<boost::thread*> threads_;
  std::vector<Work*> works_;
};

template <typename ControllerFactory>
TCPServer<ControllerFactory>::TCPServer(network::IOServicePool& isp,
                                        ControllerFactory* controller_factory)
    : server_(new network::tcp::Server(isp))
    , controller_factory_(controller_factory) {
  server_->new_connection_signal.connect(
        boost::bind(&TCPServer::HandleNewConnection, this, _1));
}

template <typename ControllerFactory>
bool TCPServer<ControllerFactory>::Open(const std::string& address,
                                        unsigned short port,
                                        size_t work_count) {
  if (!server_->Open(address, port))
    return false;

  ControllerIdGenerator id_generator = std::bind(&This::NextId, this);

  // create work threads
  threads_.reserve(work_count);
  works_.reserve(work_count);
  for (size_t i = 0; i < work_count; ++i) {
    Work* w = new Work(controller_factory_, id_generator);
    works_.push_back(w);
    boost::thread* t = new boost::thread(&Work::Process, w);
    threads_.push_back(t);
  }

  return true;
}

template <typename ControllerFactory>
bool TCPServer<ControllerFactory>::Close() {
  bool ret = server_->Close();

  // stop works
  for (typename std::vector<Work*>::iterator it = works_.begin(),
       end = works_.end(); it != end; ++it) {
    (*it)->Quit();
  }
  // stop and delete threads
  for (typename std::vector<boost::thread*>::iterator it = threads_.begin(),
       end = threads_.end(); it != end; ++it) {
    if ((*it)->joinable())
      (*it)->join();
    delete *it;
  }
  threads_.clear();
  // delete works
  for (typename std::vector<Work*>::iterator it = works_.begin(),
       end = works_.end(); it != end; ++it) {
    delete *it;
  }
  works_.clear();

  return ret;
}

template <typename ControllerFactory>
uint32_t TCPServer<ControllerFactory>::NextId() {
  static uint32_t id = 0;
  id++;
  do {
    id = id % 0xffffffff;
  } while (ContainsController(id));
  return id;
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_TCP_SERVER_H
