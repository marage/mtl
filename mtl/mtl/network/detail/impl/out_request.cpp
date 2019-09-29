#include "mtl/network/out_request.hpp"
#include "mtl/network/protocol.hpp"
#include "mtl/network/detail/byte_order.hpp"

namespace mtl {
namespace network {

OutRequest::OutRequest(uint32_t cmd, uint32_t seq, uint16_t begin)
    : OutStream(begin)
{
    writeVersion(kVersion);
    writeCommand(cmd);
    writeSequence(seq);
    writeLength(kHeaderLength);
    skip(kHeaderLength, kSkipCurrent);
}

OutRequest::OutRequest(const SharedBuffer& buffer, uint16_t size, uint16_t begin)
    : OutStream(buffer, size, begin)
{
    skip(kHeaderLength, kSkipCurrent);
}

uint16_t OutRequest::length() const
{
    uint16_t v;
    char* p = reinterpret_cast<char*>(buffer().get()) + begin();
    memcpy(&v, p, sizeof(uint16_t));
    return NetworkToHost16(v);
}

uint8_t OutRequest::version() const
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength;
    return *((uint8_t*)p);
}

uint32_t OutRequest::command() const
{
    uint32_t v;
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength + kVerFieldLength;
    memcpy(&v, p, sizeof(uint32_t));
    return NetworkToHost32(v);
}

uint8_t OutRequest::option() const
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength + kVerFieldLength
              + kCommandFieldLength;
    return *((uint8_t*)p);
}

uint32_t OutRequest::sequence() const
{
    uint32_t v;
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength
              + kVerFieldLength + kCommandFieldLength + kOptionFieldLength;
    memcpy(&v, p, sizeof(uint32_t));
    return NetworkToHost32(v);
}

const char* OutRequest::data() const
{
    return (char*)buffer().get() + begin() + kHeaderLength;
}

uint16_t OutRequest::dataSize() const
{
    return (size() - begin() - kHeaderLength);
}

OutRequest& OutRequest::writeLength(uint16_t len)
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin();
    uint16_t v = HostToNetwork16(len);
    memcpy(p, &v, sizeof(uint16_t));
    return (*this);
}

OutRequest& OutRequest::writeVersion(uint8_t ver)
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength;
    *(reinterpret_cast<uint8_t*>(p)) = ver;
    return (*this);
}

OutRequest& OutRequest::writeCommand(uint32_t cmd)
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength + kVerFieldLength;
    uint32_t v = HostToNetwork32(cmd);
    memcpy(p, &v, sizeof(uint32_t));
    return (*this);
}

OutRequest& OutRequest::writeOption(uint8_t option)
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength + kVerFieldLength
              + kCommandFieldLength;
    *(reinterpret_cast<uint8_t*>(p)) = option;
    return (*this);
}

OutRequest& OutRequest::writeSequence(uint32_t seq)
{
    char* p = reinterpret_cast<char*>(buffer().get()) + begin() + kLenFieldLength + kVerFieldLength
              + kCommandFieldLength + kOptionFieldLength;
    uint32_t v = HostToNetwork32(seq);
    memcpy(p, &v, sizeof(uint32_t));
    return (*this);
}
} // namespace network
} // namespace mtl
