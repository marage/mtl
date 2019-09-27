#ifndef MTL_FRAMEWORK_CORE_TCP_CLIENT_H
#define MTL_FRAMEWORK_CORE_TCP_CLIENT_H
#include <boost/thread.hpp>
#include "mtl/network/tcp/client.hpp"
#include "mtl/framework/log/log.h"
#include "framework.h"
#include "heart_beat_task.h"

namespace mtl {
namespace framework {
namespace core {

template <typename UnitFactory>
class TCPClient : private boost::noncopyable
{
public:
    TCPClient(boost::asio::io_context& io_context, UnitFactory* unit_factory);
    ~TCPClient() {}

    bool open(const std::string& host, const std::string& service, size_t work_count = 1);
    void close();

protected:
    network::tcp::Client& client() { return *client_; }

private:
    void handleNewConnection(const boost::system::error_code& ec);
    void handleArrival(InRequest& ireq);

    typedef typename UnitFactory::Unit Unit;
    typedef typename UnitFactory::unit_ptr unit_ptr;

    UnitFactory* unit_factory_;
    network::tcp::client_ptr client_;
    size_t work_count_;
    std::vector<boost::thread*> threads_;
    typename std::vector<unit_ptr> works_;
};

template <typename UnitFactory>
TCPClient<UnitFactory>::TCPClient(boost::asio::io_context& io_context, UnitFactory* unit_factory)
    : unit_factory_(unit_factory), client_(new network::tcp::Client(io_context)), work_count_(0)
{
    client_->new_connection_signal.connect(
                boost::bind(&TCPClient::handleNewConnection, this, _1));
    client_->packet_arrival_signal.connect(
                boost::bind(&TCPClient::handleArrival, this, _1));
}

template <typename UnitFactory>
bool TCPClient<UnitFactory>::open(const std::string& host, const std::string& service, size_t work_count)
{
    work_count_ = work_count;
    return client_->open(host, service);
}

template <typename UnitFactory>
void TCPClient<UnitFactory>::close()
{
    // stop works
    for (typename std::vector<unit_ptr>::iterator it = works_.begin(), end = works_.end();
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
    works_.clear();
    // close the connection
    client_->close();
}

template <typename UnitFactory>
void TCPClient<UnitFactory>::handleNewConnection(const boost::system::error_code& ec)
{
    if (!ec) {
        // create work threads
        threads_.reserve(work_count_);
        works_.reserve(work_count_);
        for (size_t i = 0; i < work_count_; ++i) {
            unit_ptr w = unit_factory_->createUnit(0, client_, Unit::PROCESS_LOOP);
            works_.push_back(w);
            boost::thread* t = new boost::thread(&Unit::process, w);
            threads_.push_back(t);
        }
        // heart-beat task
        {
            task_ptr t(new HeartBeatTask(client_, HEARTBEAT_TIMEOUT));
            works_.at(0)->addTask(t);
        }
        // log
        auto lg = log::global_logger::get();
        BOOST_LOG_FUNCTION()
        BOOST_LOG_SEV(lg, log::SL_WARNING) << work_count_ << "connected.";
    } else {
        // log
        auto lg = log::global_logger::get();
        BOOST_LOG_FUNCTION()
        BOOST_LOG_SEV(lg, log::SL_WARNING) << ec.message() << ec.value();
    }
}

template <typename UnitFactory>
void TCPClient<UnitFactory>::handleArrival(InRequest& ireq)
{
    size_t idx = 0;
    size_t min = 0xffffffff;
    for (size_t i = 0; i < works_.size(); ++i) {
        if (works_.at(i)->requestCount() < min) {
            min = works_.at(i)->requestCount();
            idx = i;
        }
    }
    works_[idx]->pushRequest(ireq);
}

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_TCP_CLIENT_H
