#ifndef __SENSORNETWORKESPNOW_H_
#define __SENSORNETWORKESPNOW_H_
#include <string>
#include <vector>
using std::string;
using std::vector;
class String;

String getMacAddress();
inline string sfmt(const char *format, ...); 
float avgAnalogRead(int pin, int avg);

class SchemaList;
class ConfigItem { 
protected:
    static vector<ConfigItem *> ciList;
    bool isOutput = false;
public:    
    ConfigItem(std::string n = "") : name(n) { ciList.push_back(this); }
    virtual void begin() {}
    virtual string makeSchema() = 0;
    virtual string makeReport() = 0;
    virtual void setValue(const string &s) {}
    string name = "";
    string result = "";
    float asFloat() { return strtof(result.c_str(), NULL); }
    string asString() { return result; }

    // global handler functions, move to new class
    typedef ConfigItem *(*SchemaParserFunc)(const string &);
    static vector<SchemaParserFunc> parserList;

    static ConfigItem *parseSchema(const string &s) {
        string name = s.substr(0, s.find("="));
        string sch = s.substr(s.find("=") + 1); 
        for(auto i : parserList) { 
            ConfigItem *p = i(sch);
            if (p) {
                p->name = name;
                return p;
            }
        }
        return NULL;
    }
    struct RegisterClass { 
        RegisterClass(SchemaParserFunc a) { parserList.push_back(a); }
    };

    static string makeAllSchema() { 
        string r;
        for(auto i : ciList) { 
            r += i->name + "=" + i->makeSchema() + " ";
        }
        return r;
    }

    static string makeAllResults() { 
        string r;
        for(auto i : ciList) { 
            i->result = i->makeReport();
            r += i->name + "=" + i->result + " ";
        }
        return r;
    }

    static vector<ConfigItem *> parseAllSchema(const string &s) {
        vector<ConfigItem *> rval; 
        for (auto w : split(s, ' ')) { 
            for(auto ci : ciList) { 
                auto p = ci->parseSchema(w);
                if (p) {
                    rval.push_back(p);
                    break;
                }
            }        
        } 
        return rval;
    }

    static void parseAllResults(const string &s) {
        for (auto w : split(s, ' ')) { 
            string name = w.substr(0, w.find("="));
            string v = w.substr(w.find("=") + 1); 
            for(auto i : ciList) { 
                if (name == i->name) { 
                    i->result = v;
                    break;
                }
            }
        } 
    }

    static void parseAllSetValues(const string &s) {
        vector<ConfigItem *> rval; 
        for (auto w : split(s, ' ')) { 
            for(auto ci : ciList) {
                string name = w.substr(0, w.find("="));
                string v = w.substr(s.find("=") + 1); 
                for(auto i : ciList) { 
                    if (name == i->name) { 
                        i->setValue(v);
                        break;
                    }
                }
            }        
        } 
    }

    static string makeAllSetValues() {
        string r;
        for(auto i : ciList) { 
            if (i->isOutput) {
                r += i->name + "=" + i->result + " ";
            }
        }
        return r;
    }

    static void debugPrint() { 
        for(auto ci : ciList) { 
            printf("%s: %s\n", ci->name.c_str(), ci->result.c_str());
        }
    
    }
};
vector<ConfigItem *> ConfigItem::ciList;
vector<ConfigItem::SchemaParserFunc> ConfigItem::parserList;


class ConfigItemSchemaHash : public ConfigItem { 
public:
    ConfigItemSchemaHash() : ConfigItem("SCHASH") { isOutput = true; }
    void begin() {}
    string makeSchema() { return "SCHASH"; }
    string makeReport() { return "XXXHASHXXX"; }
    static RegisterClass reg;
};
ConfigItem::RegisterClass ConfigItemSchemaHash::reg([](const string &s)->ConfigItem * { 
    return s == "SCHASH" ? new ConfigItemSchemaHash() : NULL; 
});

class ConfigItemMillis : public ConfigItem { 
    public:
        ConfigItemMillis() : ConfigItem("MILLIS") {}
        void begin() {}
        string makeSchema() { return "MILLIS"; }
        string makeReport() { return sfmt("%d", millis()); }
        static RegisterClass reg;
    };
    ConfigItem::RegisterClass ConfigItemMillis::reg([](const string &s)->ConfigItem * { 
        return s == "MILLIS" ? new ConfigItemMillis() : NULL; 
    });
    
class ConfigItemMAC : public ConfigItem { 
public:
    ConfigItemMAC() : ConfigItem("MAC") {}
    string makeSchema() { return "MAC"; }
    string makeReport() { return getMacAddress().c_str(); }
    static RegisterClass reg;
};
ConfigItem::RegisterClass ConfigItemMAC::reg([](const string &s)->ConfigItem * { 
    return s == "MAC" ? new ConfigItemMAC() : NULL; 
});
    
class ConfigItemGit : public ConfigItem { 
public:
    ConfigItemGit() : ConfigItem("GIT") {}
    string makeSchema() { return "GIT"; }
    string makeReport() { return string(GIT_VERSION); }
    static RegisterClass reg;
};
ConfigItem::RegisterClass ConfigItemGit::reg([](const string &s)->ConfigItem * { 
    return s == "GIT" ? new ConfigItemGit() : NULL; 
});
    
class ConfigItemADC : public ConfigItem { 
    int pin;
    float scale;
public:
    ConfigItemADC(const char *n, int p, float s = 1.0) : ConfigItem(n), pin(p), scale(s) {}    
    string makeSchema() { return sfmt("ADC%d*%f", pin, scale); }
    string makeReport() { return sfmt("%f", avgAnalogRead(pin, 1024) * scale); }
    static RegisterClass reg;
};
ConfigItem::RegisterClass ConfigItemADC::reg([](const string &s)->ConfigItem * { 
    float pin, scale;
    if (sscanf(s.c_str(), "ADC%f*%f", &pin, &scale) == 2) 
        return new ConfigItemADC("", pin);
    return NULL; 
});

class ConfigItemDHT : public ConfigItem { 
    int pin;
public:
    ConfigItemDHT(const char *n, int p) : ConfigItem(n), pin(p) {}    
    void begin() {}
    string makeSchema() { return sfmt("DHT%d", pin); }
    string makeReport() { return sfmt("%f", 12.34); }
    static RegisterClass reg;
};

ConfigItem::RegisterClass ConfigItemDHT::reg([](const string &s)->ConfigItem * { 
    int pin;
    if (sscanf(s.c_str(), "DHT%d", &pin) == 1) 
        return new ConfigItemDHT("", pin);
    return NULL; 
});

class ConfigItemInput : public ConfigItem { 
    int pin, mode;
public:
    ConfigItemInput(const char *n, int p, int m) : ConfigItem(n), pin(p), mode(m) {}    
    void begin() { pinMode(pin, mode); }
    string makeSchema() {
        const char *m = "";
        if (mode == INPUT_PULLUP) m = "PU";
        if (mode == INPUT_PULLDOWN) m = "PD";  
        return sfmt("INPUT%d%s", pin, m); 
    }
    string makeReport() { return sfmt("%d", digitalRead(pin)); }
    static RegisterClass reg;
};

ConfigItem::RegisterClass ConfigItemInput::reg([](const string &s)->ConfigItem * { 
    int pin, mode = INPUT;
    if (sscanf(s.c_str(), "INPUT%d", &pin) == 1) {
        if (strstr(s.c_str(), "PU") != NULL) mode = INPUT_PULLUP;
        if (strstr(s.c_str(), "PD") != NULL) mode = INPUT_PULLDOWN;
        return new ConfigItemInput("", pin, mode);
    }
    return NULL; 
});

class ConfigItemOutput : public ConfigItem { 
    int pin, mode;
public:
    ConfigItemOutput(const char *n, int p, int m) : ConfigItem(n), pin(p), mode(m) {
        isOutput = true;
    }    
    void begin() { pinMode(pin, OUTPUT); digitalWrite(pin, mode); }
    string makeSchema() { return sfmt("OUTPUT%d,%d", pin, mode); }
    string makeReport() { return sfmt("%d", digitalRead(pin)); }
    void setValue(const string &s) { 
        sscanf("%d", s.c_str(), &mode);
        digitalWrite(pin, mode);
    }
    static RegisterClass reg;
};
ConfigItem::RegisterClass ConfigItemOutput::reg([](const string &s)->ConfigItem * { 
    int pin, mode;
    if (sscanf(s.c_str(), "OUTPUT%d,%d", &pin, &mode) == 1) {
        return new ConfigItemOutput("", pin, mode);
    }
    return NULL; 
});



//static SchemaList::Register();
#endif //__SENSORNETWORKESPNOW_H_