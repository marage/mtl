#include "mtl/utility/util.hpp"
//#include <codecvt>
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
    mtl::RC4::init(rc_key, key, key_len);
    mtl::RC4::encrypt(rc_key, data, data_len);
    return data;
}

static inline char toChar(unsigned char v)
{
    return (v > 9 ? 'A' + v - 10 : '0' + v);
}

std::string bcd(const unsigned char* data, int len)
{
    std::string s;
    if (len > 0) {
        const unsigned char* p = data;
        unsigned char* buf = new unsigned char[2*len];
        unsigned char* q = buf;
        for (int i = 0; i < len; ++i) {
            q[i*2] = toChar(*p >> 4);
            q[i*2+1] = toChar(*p & 0x0F);
            ++p;
        }
        s.assign((const char*)buf, 2*len-1);
        delete []buf;
    }
    return s;
}

int evaluateScale(const std::string& ip1, const std::string& ip2)
{
    int scale = 1;
    const char* p = ip1.c_str();
    const char* q = ip2.c_str();
    while (*p == *q && *p++ != '\0' && *q++ != '\0') {
        scale *= 2;
    }
    return scale;
}

uint64_t generateNextId()
{
    static uint64_t id = 0;
    return ++id;
}

std::string log_string;
std::string getLogString()
{
    return log_string;
}

extern void log(const char* format, ...)
{
//    va_list vl;

//    va_start(vl, format);

//    char tmp[512] = { 0 };
//    ::vsprintf(tmp, format, vl);

//    va_end(vl);

//    log_string += tmp;
//    log_string += "\r\n";
}

//std::wstring utf8ToUnicode(const std::string& src)
//{
//    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
//    return conv.from_bytes(src);
//}

//std::string unicodeToUtf8(const std::wstring& src)
//{
//    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
//    return conv.to_bytes(src);
//}

//std::wstring ansiToUnicode(const std::string& src)
//{
//    std::wstring s(src.length(), L'\0');
//    std::copy(src.begin(), src.end(), s.begin());
//    return s;
//}

//std::string unicodeToAnsi(const std::wstring& src)
//{
//    std::string s(src.length(), '\0');
//    std::copy(src.begin(), src.end(), s.begin());
//    return s;
//}

}
}
