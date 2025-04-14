#pragma once
#include "jimlib.h"

class SerialLogManager {
    DeepSleepElapsedTimer dsTime = DeepSleepElapsedTimer("/SerLogMan");
    uint32_t options;
public:
    SerialLogManager(int _options = 0) : options(_options) {}
    static const int showMillis = 0x1;
    static const int showSleep = 0x2;
    static const int showLineNr = 0x4;
    static const int showBar = 0x8;
    void setOptions(uint32_t x) { options |= x; } 
    void clearOptions(uint32_t x) { options &= ~x; } 
    void vout(const char *file, int line, const char *format, va_list args); 
    void out(const char *file, int line, const char *format, ...);
}; 

extern SerialLogManager serialLog;

#define OUT(...) serialLog.out(__FILE__, __LINE__, __VA_ARGS__)
#define LOG OUT
static inline void dbg(const char *(format), ...) { }

