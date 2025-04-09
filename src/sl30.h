#pragma once
#include <string>

// From data format described in web search "SL30 Installation Manual PDF" 

class SL30 {
public:
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
        hd *= 127 / 3;
        vd *= 127 / 3;
        return pmrrv((std::string("21") + twoenc(hd) + twoenc(vd) + twoenc(flags)).c_str());
    }
};
    
    