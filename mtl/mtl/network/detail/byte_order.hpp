/*
 * libjingle
 * Copyright 2004--2005, Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MTL_NETWORK_BYTE_ORDER_HPP
#define MTL_NETWORK_BYTE_ORDER_HPP
#include <boost/cstdint.hpp>
#include <boost/asio.hpp>

namespace mtl {
namespace network {

enum ByteOrder
{
    ORDER_NETWORK = 0,  // Default, use network byte order (big endian).
    ORDER_HOST,         // Use the native order of the host.
};

// Reading and writing of little and big-endian numbers from memory
// TODO: Add HostEndian #defines (HE)
// TODO: Consider NetworkEndian as synonym for BigEndian, for clarity in use.
// TODO: Consider creating optimized versions, such as direct read/writes of
// integers in host-endian format, when the platform supports it.

inline void Set8(void* memory, size_t offset, uint8_t v)
{
    static_cast<uint8_t*>(memory)[offset] = v;
}
inline uint8_t Get8(const void* memory, size_t offset)
{
    return static_cast<const uint8_t*>(memory)[offset];
}

inline void SetBE16(void* memory, uint16_t v)
{
    Set8(memory, 0, static_cast<uint8_t>(v >>  8));
    Set8(memory, 1, static_cast<uint8_t>(v >>  0));
}
inline void SetBE32(void* memory, uint32_t v)
{
    Set8(memory, 0, static_cast<uint8_t>(v >> 24));
    Set8(memory, 1, static_cast<uint8_t>(v >> 16));
    Set8(memory, 2, static_cast<uint8_t>(v >>  8));
    Set8(memory, 3, static_cast<uint8_t>(v >>  0));
}
inline void SetBE64(void* memory, uint64_t v)
{
    Set8(memory, 0, static_cast<uint8_t>(v >> 56));
    Set8(memory, 1, static_cast<uint8_t>(v >> 48));
    Set8(memory, 2, static_cast<uint8_t>(v >> 40));
    Set8(memory, 3, static_cast<uint8_t>(v >> 32));
    Set8(memory, 4, static_cast<uint8_t>(v >> 24));
    Set8(memory, 5, static_cast<uint8_t>(v >> 16));
    Set8(memory, 6, static_cast<uint8_t>(v >>  8));
    Set8(memory, 7, static_cast<uint8_t>(v >>  0));
}
inline uint16_t GetBE16(const void* memory)
{
    return (static_cast<uint16_t>(Get8(memory, 0)) << 8)
            | (static_cast<uint16_t>(Get8(memory, 1)) << 0);
}
inline uint32_t GetBE32(const void* memory)
{
    return (static_cast<uint32_t>(Get8(memory, 0)) << 24)
            | (static_cast<uint32_t>(Get8(memory, 1)) << 16)
            | (static_cast<uint32_t>(Get8(memory, 2)) <<  8)
            | (static_cast<uint32_t>(Get8(memory, 3)) <<  0);
}
inline uint64_t GetBE64(const void* memory)
{
    return (static_cast<uint64_t>(Get8(memory, 0)) << 56)
            | (static_cast<uint64_t>(Get8(memory, 1)) << 48)
            | (static_cast<uint64_t>(Get8(memory, 2)) << 40)
            | (static_cast<uint64_t>(Get8(memory, 3)) << 32)
            | (static_cast<uint64_t>(Get8(memory, 4)) << 24)
            | (static_cast<uint64_t>(Get8(memory, 5)) << 16)
            | (static_cast<uint64_t>(Get8(memory, 6)) <<  8)
            | (static_cast<uint64_t>(Get8(memory, 7)) <<  0);
}

inline void SetLE16(void* memory, uint16_t v)
{
    Set8(memory, 1, static_cast<uint8_t>(v >>  8));
    Set8(memory, 0, static_cast<uint8_t>(v >>  0));
}
inline void SetLE32(void* memory, uint32_t v)
{
    Set8(memory, 3, static_cast<uint8_t>(v >> 24));
    Set8(memory, 2, static_cast<uint8_t>(v >> 16));
    Set8(memory, 1, static_cast<uint8_t>(v >>  8));
    Set8(memory, 0, static_cast<uint8_t>(v >>  0));
}
inline void SetLE64(void* memory, uint64_t v)
{
    Set8(memory, 7, static_cast<uint8_t>(v >> 56));
    Set8(memory, 6, static_cast<uint8_t>(v >> 48));
    Set8(memory, 5, static_cast<uint8_t>(v >> 40));
    Set8(memory, 4, static_cast<uint8_t>(v >> 32));
    Set8(memory, 3, static_cast<uint8_t>(v >> 24));
    Set8(memory, 2, static_cast<uint8_t>(v >> 16));
    Set8(memory, 1, static_cast<uint8_t>(v >>  8));
    Set8(memory, 0, static_cast<uint8_t>(v >>  0));
}
inline uint16_t GetLE16(const void* memory)
{
    return (static_cast<uint16_t>(Get8(memory, 1)) << 8)
            | (static_cast<uint16_t>(Get8(memory, 0)) << 0);
}
inline uint32_t GetLE32(const void* memory)
{
    return (static_cast<uint32_t>(Get8(memory, 3)) << 24)
            | (static_cast<uint32_t>(Get8(memory, 2)) << 16)
            | (static_cast<uint32_t>(Get8(memory, 1)) <<  8)
            | (static_cast<uint32_t>(Get8(memory, 0)) <<  0);
}
inline uint64_t GetLE64(const void* memory)
{
    return (static_cast<uint64_t>(Get8(memory, 7)) << 56)
            | (static_cast<uint64_t>(Get8(memory, 6)) << 48)
            | (static_cast<uint64_t>(Get8(memory, 5)) << 40)
            | (static_cast<uint64_t>(Get8(memory, 4)) << 32)
            | (static_cast<uint64_t>(Get8(memory, 3)) << 24)
            | (static_cast<uint64_t>(Get8(memory, 2)) << 16)
            | (static_cast<uint64_t>(Get8(memory, 1)) <<  8)
            | (static_cast<uint64_t>(Get8(memory, 0)) <<  0);
}

// Check if the current host is big endian.
inline bool IsHostBigEndian()
{
    static const int number = 1;
    return (0 == *reinterpret_cast<const char*>(&number));
}

inline uint16_t HostToNetwork16(uint16_t n)
{
    return htons(n);
}

inline uint32_t HostToNetwork32(uint32_t n)
{
    return htonl(n);
}

inline uint64_t HostToNetwork64(uint64_t n)
{
    // If the host is little endian, GetBE64 converts n to big network endian.
    return IsHostBigEndian() ? n : GetBE64(&n);
}

inline boost::uint16_t NetworkToHost16(uint16_t n)
{
    return ntohs(n);
}

inline uint32_t NetworkToHost32(uint32_t n)
{
    return ntohl(n);
}

inline uint64_t NetworkToHost64(uint64_t n)
{
    // If the host is little endian, GetBE64 converts n to little endian.
    return IsHostBigEndian() ? n : GetBE64(&n);
}

} // namespace network
} // namespace mtl

#endif  // MTL_NETWORK_BYTE_ORDER_HPP
