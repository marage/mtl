#include "mtl/network/singleton_buffer_pool.hpp"
#include <boost/assert.hpp>

template<> mtl::network::SingletonBufferPool * mtl::Singleton<mtl::network::SingletonBufferPool>::ms_Singleton = nullptr;

namespace mtl {
namespace network {

class BufferPoolDeallocator {
public:
    inline BufferPoolDeallocator() {}

    static BufferPoolDeallocator& instance()
    {
        static BufferPoolDeallocator inst;
        return inst;
    }

    inline void operator()(char*p)
    {
        SingletonBufferPool::getSingletonPtr()->freeBuffer(p);
    }
};

SingletonBufferPool& SingletonBufferPool::getSingleton()
{
    assert(ms_Singleton);
    return *ms_Singleton;
}

SingletonBufferPool* SingletonBufferPool::getSingletonPtr()
{
    return ms_Singleton;
}

SingletonBufferPool::SingletonBufferPool()
    : fixed_pool_(1024)
{
}

SingletonBufferPool::~SingletonBufferPool()
{
}

bool SingletonBufferPool::isFrom(void* p)
{
    bool ok = false;
    mutex_.lock();
    if (floating_pool_.isFrom(p)) {
        ok = true;
    } else if (fixed_pool_.is_from(p)) {
        ok = true;
    }
    mutex_.unlock();
    return ok;
}

SharedBuffer SingletonBufferPool::allocBuffer(unsigned int size)
{
    char* p = nullptr;
    mutex_.lock();
    if (size == 1024) {
        p = static_cast<char*>(fixed_pool_.malloc());
    }
    if (!p) {
        p = static_cast<char*>(floating_pool_.allocBuffer(size));
    }
    mutex_.unlock();
    return SharedBuffer(p, BufferPoolDeallocator::instance());
}

void SingletonBufferPool::freeBuffer(void* p)
{
    mutex_.lock();
    if (floating_pool_.isFrom(p)) {
        floating_pool_.freeBuffer(p);
    } else if (fixed_pool_.is_from(p)) {
        fixed_pool_.free(p);
    }
    mutex_.unlock();
}

void SingletonBufferPool::releaseMemory()
{
    floating_pool_.releaseMemory();
    fixed_pool_.release_memory();
}

} // namespace network
} // namespace mtl
