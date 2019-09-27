#include "mtl/encrypt/rc4.hpp"

namespace mtl {

void RC4::init(unsigned char s[256], const unsigned char* key, int len)
{
    int i = 0, j = 0, k[256] = { 0 };
    unsigned char tmp = 0;
    for (i = 0; i < 256; i++) {
        s[i] = i;
        k[i] = key[i % len];
    }
    for (i = 0; i < 256; i++) {
        j = (j + s[i] + k[i]) % 256;
        tmp = s[i];
        s[i] = s[j];
        s[j] = tmp;
    }
}

void RC4::encrypt(unsigned char s[256], unsigned char* data, int len)
{
    int x = 0, y = 0, t = 0, i = 0;
    unsigned char tmp;
    for (i = 0; i < len; i++) {
        x = (x + 1) % 256;
        y = (y + s[x]) % 256;
        tmp = s[x];
        s[x] = s[y];
        s[y] = tmp;
        t = (s[x] + s[y]) % 256;
        data[i] ^= s[t];
    }
}
}
