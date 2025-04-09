#include "serialLog.h" 
SerialLogManager serialLog(0xff);

void SerialLogManager::out(const char *format, ...) { 
    va_list args;
    va_start(args, format);
    string s;
    if (options & showSleep) s += sfmt("%09.1f ", dsTime.millis()/1000.0);
    if (options & showMillis) s += sfmt("%07.3f ", millis()/1000.0);
    if (options & showLineNr) {
        string ln = sfmt("%s,%d ", basename(file), line);
        const int width = 20;
        if (ln.size() < width) {
            ln.insert(ln.end(), width - ln.size(), ' '); 
        } else {
            ln.erase(ln.begin(), ln.begin() + ln.size() - width);
        }
        s += ln;
    }
    if (options & showBar) s += "| ";
    s += vsfmt(format, args);
    if (strchr(format, '\n') == NULL) 
        s += '\n'; 
    printf("%s", s.c_str());
    fflush(stdout);
    va_end(args);
};
