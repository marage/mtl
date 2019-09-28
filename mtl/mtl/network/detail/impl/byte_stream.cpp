#include "mtl/network/detail/byte_stream.hpp"
#include "mtl/network/singleton_buffer_pool.hpp"

namespace mtl {
namespace network {

ByteStream::Piece::Piece(const SharedBuffer& buf, size_t data_size)
  : block(buf), cursor(0), size(data_size) {
}

size_t ByteStream::Piece::Read(void* buf, size_t count) {
  size_t bytes_of_read = 0;
  size_t left = size - cursor;
  if (left > 0) {
    bytes_of_read = ((left > count) ? count : left);
    memcpy(buf, ((char*)block.get() + cursor), bytes_of_read);
    cursor += bytes_of_read;
  }
  return bytes_of_read;
}

size_t ByteStream::Piece::Peek(void* buf, size_t count) {
  size_t bytes_of_peek = 0;
  size_t left = size - cursor;
  if (left > 0) {
    bytes_of_peek = ((left > count) ? count : left);
    memcpy(buf, ((char*)block.get() + cursor), bytes_of_peek);
  }
  return bytes_of_peek;
}

size_t ByteStream::Length() const {
  size_t len = 0;
  std::list<Piece>::const_iterator pos = pieces_.begin();
  std::list<Piece>::const_iterator end = pieces_.end();
  for (; pos != end; pos++) {
    len += (*pos).Length();
  }
  return len;
}

size_t ByteStream::Read(void* buf, size_t count) {
  size_t bytes_of_total = 0;
  size_t bytes_to_read = count;
  size_t bytes_of_read = 0;
  while ((bytes_to_read > 0) && (pieces_.size() > 0)) {
    Piece& p = *pieces_.begin();
    bytes_of_read = p.Read(((char*)buf + bytes_of_total), bytes_to_read);
    bytes_to_read -= bytes_of_read;
    bytes_of_total += bytes_of_read;
    if (p.Empty()) {
      pieces_.erase(pieces_.begin());
    }
  }
  return bytes_of_total;
}

size_t ByteStream::Peek(void* buf, size_t count) {
  size_t bytes_of_total = 0;
  size_t bytes_to_peek = count;
  size_t bytes_of_peek = 0;
  std::list<Piece>::iterator pos = pieces_.begin();
  std::list<Piece>::iterator end = pieces_.end();
  for (; ((bytes_to_peek > 0) && (pos != end)); ++pos) {
    bytes_of_peek = (*pos).Peek(((char*)buf + bytes_of_total), bytes_to_peek);
    bytes_to_peek -= bytes_of_peek;
    bytes_of_total += bytes_of_peek;
  }
  return bytes_of_total;
}
}
}
