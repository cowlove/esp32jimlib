#include "serialLog.h" 
SerialLogManager serialLog(0xff);

void SerialLogManager::vout(const char *file, int line, const char *format, va_list args) { 
    return out(file, line, vsfmt(format, args));
}

void SerialLogManager::out(const char *file, int line, const char *format, ...) {
    va_list args;
    va_start(args, format);
    vout(file, line, format, args);
    va_end(args);
};

void SerialLogManager::out(const char *file, int line, const string &l) { 
    string s;
    if (options & showSleep) s += sfmt("%06.1f ", dsTime.millis()/1000.0);
    if (options & showMillis) s += sfmt("%06.3f ", millis()/1000.0);
    if (options & showLineNr) {
        string ln = sfmt("%s,%d ", basename(file), line);
        const int width = 18;
        if (ln.size() < width) {
            ln.insert(ln.end(), width - ln.size(), ' '); 
        } else {
            ln.erase(ln.begin(), ln.begin() + ln.size() - width);
        }
        s += ln;
    }
    if (options & showBar) s += "| ";
    s += l;
    if (s.find('\n') == string::npos)
        s += '\n'; 

    printf("%s", s.c_str());
    fflush(stdout);
}

