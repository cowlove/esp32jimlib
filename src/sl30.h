#pragma once
#include <string>

// From data format described in web search "SL30 Installation Manual PDF" 

class SL30 {
public:
    static constexpr unsigned char FLAG_FROM = 1 << 2;
    static constexpr unsigned char FLAG_TO = 1 << 3;
    static constexpr unsigned char FLAG_NAV_SUPER = 1 << 6;
    static constexpr unsigned char FLAG_NAV_VALID = 1 << 7;

    double lastCdiHd = 0;
    double lastCdiVd = 0;
    unsigned char lastCdiHdByte = 0;
    unsigned char lastCdiVdByte = 0;
    unsigned char lastCdiFlags = 0;

    std::string twoenc(unsigned char x) {
        char r[3];
        r[0] = (((x & 0xf0) >> 4) + 0x30);
        r[1] = (x & 0xf) + 0x30;
        r[2] = 0;
        return std::string(r);
    }
    int chksum(const std::string& r) {
        int sum = 0;
        const char* s = r.c_str();
        while (*s)
                sum += *s++;
        return sum & 0xff;
    }
    void open() {}
    std::string pmrrv(const char *r) {
        return std::string("$PMRRV") + r + twoenc(chksum(r)) + "\r\n";
        //Serial2.write(s.c_str());
        //Serial.printf("G5: %s", s.c_str());
        //Serial.write(s.c_str());
    }
    std::string setCDI(double hd, double vd) {
        int flags = 0b11111010;
        return setCDI(hd, vd, flags);
    }
    std::string setCDI(double hd, double vd, bool to, bool from) {
        unsigned char flags = FLAG_NAV_SUPER | FLAG_NAV_VALID;
        if (to)
            flags |= FLAG_TO;
        if (from)
            flags |= FLAG_FROM;
        return setCDI(hd, vd, flags);
    }
    std::string setCDI(double hd, double vd, unsigned char flags) {
        lastCdiHd = hd;
        lastCdiVd = vd;
        hd *= 127 / 3;
        vd *= 127 / 3;
        lastCdiHdByte = (unsigned char)hd;
        lastCdiVdByte = (unsigned char)vd;
        lastCdiFlags = flags;
        return pmrrv((std::string("21") + twoenc(hd) + twoenc(vd) + twoenc(flags)).c_str());
    }
};
    
