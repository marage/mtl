//
// context_pool.cpp
// ~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "mtl/network/context_pool.hpp"
#include <stdexcept>
#include <boost/bind.hpp>

template<> mtl::network::ContextPool* mtl::Singleton<mtl::network::ContextPool>::ms_Singleton = nullptr;

namespace mtl {
namespace network {

ContextPool& ContextPool::getSingleton()
{
    assert(ms_Singleton);
    return *ms_Singleton;
}

ContextPool* ContextPool::getSingletonPtr()
{
    return ms_Singleton;
}

ContextPool::ContextPool(std::size_t pool_size)
    : next_(0)
{
    if (pool_size == 0) {
        throw std::runtime_error("io_context_pool size is 0");
    }

    // Give all the io_contexts work to do so that their run() functions will not
    // exit until they are explicitly stopped.
    for (std::size_t i = 0; i < pool_size; ++i) {
        io_context_ptr io_service(new boost::asio::io_context);
        work_ptr work(new boost::asio::io_context::work(*io_service));
        contexts_.push_back(io_service);
        work_.push_back(work);
    }
}

void ContextPool::run()
{
    // Create a pool of threads to run all of the io_contexts.
    for (std::size_t i = 0; i < contexts_.size(); ++i) {
        boost::shared_ptr<boost::thread> thread(new boost::thread(
                                                    boost::bind(&boost::asio::io_context::run, contexts_[i])));
        threads_.push_back(thread);
    }
}

void ContextPool::joinAll()
{
    // Wait for all threads in the pool to exit.
    for (std::size_t i = 0; i < threads_.size(); ++i) {
        threads_[i]->join();
    }

    threads_.clear();
}

void ContextPool::stop()
{
    // Explicitly stop all io_contexts.
    for (std::size_t i = 0; i < contexts_.size(); ++i) {
        contexts_[i]->stop();
    }
}

boost::asio::io_context& ContextPool::getContext()
{
    // Use a round-robin scheme to choose the next io_context to use.
    boost::asio::io_context& context = *contexts_[next_];
    ++next_;
    if (next_ == contexts_.size()) {
        next_ = 0;
    }
    return context;
}

} // namespace network
} // namespace mtl
