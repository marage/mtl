#ifndef MTL_NETWORK_BYTE_STREAM_HPP
#define MTL_NETWORK_BYTE_STREAM_HPP
#include <boost/noncopyable.hpp>
#include <list>
#include "mtl/network/shared_buffer.hpp"

namespace mtl {
namespace network {

class ByteStream : public boost::noncopyable
{
private:
    struct Piece
    {
        Piece(const SharedBuffer& buf, size_t data_size);

        inline bool empty() const { return (cursor >= size); }
        inline size_t length() const { return (size - cursor); }

        size_t read(void* buf, size_t count);
        size_t peek(void* buf, size_t count);

        SharedBuffer block;
        size_t cursor;
        size_t size;
    };

public:
    ByteStream() {}

    inline bool empty() const { return pieces_.empty(); }

    size_t length() const;

    size_t read(void* buf, size_t count);
    size_t peek(void* buf, size_t count);

    inline void write(const SharedBuffer& buf, size_t data_size)
    {
        pieces_.push_back(Piece(buf, data_size));
    }

    inline void clear()
    {
        pieces_.clear();
    }

private:
    std::list<Piece> pieces_;
};

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_BYTE_STREAM_HPP
