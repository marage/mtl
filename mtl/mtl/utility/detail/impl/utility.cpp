#include "mtl/utility/utility.hpp"
#include <codecvt>
#include <locale>
#include <algorithm>
#include <stdarg.h>
#include "mtl/encrypt/rc4.hpp"

namespace mtl {
namespace util {

unsigned char* Encrypt(unsigned char* data, int data_len,
                       const unsigned char* key, int key_len) {
  unsigned char rc_key[256];
  mtl::RC4::Init(rc_key, key, key_len);
  mtl::RC4::Encrypt(rc_key, data, data_len);
  return data;
}

static inline char toChar(unsigned char v) {
  return (v > 9 ? char('A' + v - 10) : char('0' + v));
}

std::string Bcd(const unsigned char* data, int len) {
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

int EvaluateScale(const std::string& ip1, const std::string& ip2) {
  int scale = 1;
  const char* p = ip1.c_str();
  const char* q = ip2.c_str();
  while (*p == *q && *p++ != '\0' && *q++ != '\0') {
    scale *= 2;
  }
  return scale;
}

uint64_t GenerateNextId() {
  static uint64_t id = 0;
  return ++id;
}

std::wstring Utf8ToUnicode(const std::string& src) {
  std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
  return conv.from_bytes(src);
  //  (void)src;
  //  return std::wstring(L"");
}

std::string UnicodeToUtf8(const std::wstring& src) {
  std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
  return conv.to_bytes(src);
  //  (void)src;
  //  return std::string("");
}

std::wstring AnsiToUnicode(const std::string& src) {
  std::wstring s(src.length(), L'\0');
  std::copy(src.begin(), src.end(), s.begin());
  return s;
}

std::string UnicodeToAnsi(const std::wstring& src) {
  std::string s(src.length(), '\0');
  std::copy(src.begin(), src.end(), s.begin());
  return s;
}

void StringReplace(std::string& str, const std::string& src,
                   const std::string& dest) {
  std::string::size_type slen = src.length();
  std::string::size_type dlen = dest.length();
  std::string::size_type pos = 0;
  std::string::size_type off = 0;
  while ((pos = str.find(src, off)) != std::string::npos) {
    str.replace(pos, slen, dest);
    off = pos + dlen;
  }
}

bool HasEspecialCharacter(const std::string& s) {
  return ((s.find('\\') != std::string::npos)
          || (s.find('\'') != std::string::npos)
          || (s.find('\"') != std::string::npos));
}

const std::string& ToMysqlString(std::string& s) {
  if (HasEspecialCharacter(s)) {
    StringReplace(s, "\\", "\\\\");
    StringReplace(s, "\'", "\\\'");
    StringReplace(s, "\"", "\\\"");
  }
  return s;
}

const std::string& ToRawString(std::string& s) {
  if (HasEspecialCharacter(s)) {
    StringReplace(s, "\\\\", "\\");
    StringReplace(s, "\\\'", "\'");
    StringReplace(s, "\\\"", "\"");
  }
  return s;
}

}
}
