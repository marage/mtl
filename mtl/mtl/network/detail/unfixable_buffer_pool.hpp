#ifndef MTL_NETWORK_UNFIXABLE_BUFFER_POOL_HPP
#define MTL_NETWORK_UNFIXABLE_BUFFER_POOL_HPP
#include <boost/noncopyable.hpp>
#include <map>
#include "mtl/mtl.hpp"

namespace mtl {
namespace network {

class UnfixableBufferPool : public boost::noncopyable
{
public:
    UnfixableBufferPool();
    ~UnfixableBufferPool();

    inline bool isFrom(void* p) const
    {
        return (valid_ptr_list_.find(p) != valid_ptr_list_.end());
    }

    void* malloc(std::size_t size);
    void free(void* p);

    void releaseMemory();

private:
    struct Node
    {
        Node* next;
    };

    struct SizeNode
    {
        uint32_t size;
        Node* head;
        Node* tail;
        SizeNode* next;
    };

    void* getBuffer(std::size_t size);
    SizeNode* getNode(std::size_t size);

    SizeNode* node_list_;
    std::map<void*, void*> valid_ptr_list_;
};

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_UNFIXABLE_BUFFER_POOL_HPP