#pragma once
#include <cstddef>
class Preferences {
public:
 bool begin(const char*,bool) { return true; }
 size_t putBytes(const char*,const void*,size_t n) { return n; }
 size_t getBytes(const char*,void*,size_t) { return 0; }
};
