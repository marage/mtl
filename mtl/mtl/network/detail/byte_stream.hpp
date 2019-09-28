#ifndef MTL_NETWORK_BYTE_STREAM_HPP
#define MTL_NETWORK_BYTE_STREAM_HPP
#include <boost/noncopyable.hpp>
#include <list>
#include "mtl/network/shared_buffer.hpp"

namespace mtl {
namespace network {

class ByteStream : public boost::noncopyable {
public:
  ByteStream() {}

  inline bool Empty() const { return pieces_.empty(); }

  size_t Length() const;

  size_t Read(void* buf, size_t count);
  size_t Peek(void* buf, size_t count);

  inline void Write(const SharedBuffer& buf, size_t data_size);
  inline void Clear() {
    pieces_.clear();
  }

private:
  struct Piece {
    Piece(const SharedBuffer& buf, size_t data_size);

    inline bool Empty() const { return (cursor >= size); }
    inline size_t Length() const { return (size - cursor); }

    size_t Read(void* buf, size_t count);
    size_t Peek(void* buf, size_t count);

    SharedBuffer block;
    size_t cursor;
    size_t size;
  };

  std::list<Piece> pieces_;
};

inline void ByteStream::Write(const SharedBuffer& buf, size_t data_size) {
  pieces_.push_back(Piece(buf, data_size));
}

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_BYTE_STREAM_HPP
