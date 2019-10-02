#include "mtl/utility/util.hpp"
#include <codecvt>
#include <locale>
#include <algorithm>
#include <stdarg.h>
#include "mtl/encrypt/rc4.hpp"

namespace mtl {
namespace util {

unsigned char* encrypt(unsigned char* data, int data_len,
                       const unsigned char* key, int key_len)
{
    unsigned char rc_key[256];
    rc4::init(rc_key, key, key_len);
    rc4::encrypt(rc_key, data, data_len);
    return data;
}

static inline char toChar(unsigned char v)
{
    return (v > 9 ? char('A' + v - 10) : char('0' + v));
}

std::string bcd(const unsigned char* data, int len)
{
    std::string s;
    if (len > 0) {
        const unsigned char* p = data;
        unsigned char* buf = new unsigned char[2 * len];
        unsigned char* q = buf;
        for (int i = 0; i < len; ++i) {
            q[i*2] = toChar(*p >> 4);
            q[i*2+1] = toChar(*p & 0x0F);
            ++p;
        }
        s.assign(reinterpret_cast<const char*>(buf), 2 * len - 1);
        delete []buf;
    }
    return s;
}

int evaluateScore(const std::string& ip1, const std::string& ip2)
{
    int score = 1;
    const char* p = ip1.c_str();
    const char* q = ip2.c_str();
    while (*p == *q && *p++ != '\0' && *q++ != '\0') {
        score *= 2;
    }
    return score;
}

std::wstring utf8ToUnicode(const std::string& src)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
    return conv.from_bytes(src);
}

std::string unicodeToUtf8(const std::wstring& src)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
    return conv.to_bytes(src);
}

std::wstring ansiToUnicode(const std::string& src)
{
    std::wstring s(src.length(), L'\0');
    std::copy(src.begin(), src.end(), s.begin());
    return s;
}

std::string unicodeToAnsi(const std::wstring& src)
{
    std::string s(src.length(), '\0');
    std::copy(src.begin(), src.end(), s.begin());
    return s;
}

}
}
