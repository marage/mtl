//
// io_context_pool.hpp
// ~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MTL_NETWORK_CONTEXT_POOL_HPP
#define MTL_NETWORK_CONTEXT_POOL_HPP
#include <vector>
#include <boost/asio/io_context.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include "mtl/mtl.hpp"
#include "mtl/singleton/singleton.hpp"

namespace mtl {
namespace network {

/// A pool of io_service objects.
class MTL_EXPORT ContextPool
    : public Singleton<ContextPool>, private boost::noncopyable
{
public:
    /// Construct the io_service pool.
    explicit ContextPool(std::size_t pool_size);

    static ContextPool& getSingleton();
    static ContextPool* getSingletonPtr();

    /// Run all io_service objects in the pool.
    void run();

    /// Join all io_service objects in the pool.
    void joinAll();

    /// Stop all io_service objects in the pool.
    void stop();

    /// Get an io_context to use.
    boost::asio::io_context& getContext();

private:
    typedef boost::shared_ptr<boost::asio::io_context> io_context_ptr;
    typedef boost::shared_ptr<boost::asio::io_context::work> work_ptr;

    /// The pool of io_contexts.
    std::vector<io_context_ptr> contexts_;

    /// The work that keeps the io_services running.
    std::vector<work_ptr> work_;

    /// A pool of threads to run all of the io_services.
    std::vector<boost::shared_ptr<boost::thread>> threads_;

    /// The next io_service to use for a connection.
    std::size_t next_;
};

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_CONTEXT_POOL_HPP
