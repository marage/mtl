#ifndef MTL_NETWORK_PROTOCOL_HPP
#define MTL_NETWORK_PROTOCOL_HPP
#include "mtl/mtl.hpp"

namespace mtl {
namespace network {

// version
constexpr uint8_t VERSION = 0x01;

// format: len | ver | common | option | seq
constexpr int LEN_FIELD_LENGTH = sizeof(uint16_t);
constexpr int VER_FIELD_LENGTH = sizeof(uint8_t);
constexpr int COMMAND_FIELD_LENGTH = sizeof(uint32_t);
constexpr int OPTION_FIELD_LENGTH = sizeof(uint8_t);
constexpr int SEQ_FIELD_LENGTH = sizeof(uint32_t);
constexpr int HEADER_LENGTH = LEN_FIELD_LENGTH + VER_FIELD_LENGTH + COMMAND_FIELD_LENGTH
                              + OPTION_FIELD_LENGTH + SEQ_FIELD_LENGTH;

enum SkipPosition
{
    SKIP_BEGIN,
    SKIP_CURRENT,
    SKIP_END
};

} // namespace network
} // namespace mtl

#endif // MTL_NETWORK_PROTOCOL_HPP
