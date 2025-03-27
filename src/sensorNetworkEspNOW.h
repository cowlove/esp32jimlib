#ifndef __SENSORNETWORKESPNOW_H_
#define __SENSORNETWORKESPNOW_H_
#include "espNowMux.h"
#include "reliableStream.h"
#include "Arduino_CRC32.h" 
#ifndef CSIM
#include "DHT.h"
#else 
struct DHT {
    DHT(int, int) {}
    void begin() {}
    float readTemperature() { return 27.01; }
    float readHumidity() { return 57.02; }
};
#define DHT22 0

#endif

#include <string>
#include <vector>
using std::string;
using std::vector;

class RemoteSensorModule;

class RemoteSensorProtocol {
protected: 
    Arduino_CRC32 crc32;
    struct {
        const string ACK = "ACK";
        const string MAC = "MAC";
        const string SERVER = "SERVER";
        const string SCHASH = "SCHASH";
        const string UPDATENOW = "UPDATENOW";
        const string SLEEP = "SLEEP";
    } specialWords;
    //TODO
    class LineMap : public std::map<string,string> {
    public:
        bool contains(const string &s) { 
            return find(s) != end();
        }
    };
    LineMap parseLine(const string &s) {
        LineMap rval;
        for(auto w : split(s, ' ')) { 
            if (s.find("=") != string::npos) {
                string name = w.substr(0, w.find("="));
                string val = w.substr(w.find("=") + 1);
                rval[name] = val;
            } else { 
                rval[s] = "";
            }
        }
        return rval;
    }
};

class Sensor { 
friend RemoteSensorModule;
protected:
    bool isOutput = false;
public:    
    Sensor(RemoteSensorModule *p = NULL, std::string n = "");
    virtual void begin() {}
    virtual string makeSchema() = 0;
    virtual string makeReport() = 0;
    virtual void setValue(const string &s) {}
    string name = "";
    string result = "";
    float asFloat() { return strtof(result.c_str(), NULL); }
    string asString() { return result; }
};

class SchemaParser : protected RemoteSensorProtocol { 
public:
    typedef Sensor *(*ParserFunc)(const string &);
    static vector<ParserFunc> parserList;

    static vector<Sensor *> parseSchema(const string &s) {
        vector<Sensor *> rval; 
        for (auto w : split(s, ' ')) { 
            string name = w.substr(0, w.find("="));
            string sch = w.substr(w.find("=") + 1); 
            for(auto i : parserList) { 
                Sensor *p = i(sch);
                if (p) {
                    p->name = name;
                    rval.push_back(p);
                    break;
                }
            }
        }
        return rval;
    }
    struct RegisterClass { 
        RegisterClass(ParserFunc a) { parserList.push_back(a); }
    };
};
vector<SchemaParser::ParserFunc> SchemaParser::parserList;

class RemoteSensorServer;
class RemoteSensorClient;
    
class RemoteSensorModule : public RemoteSensorProtocol {
    vector<Sensor *> sensors;
    vector<const char*> requiredFields = {"SCHASH=SCHASH", "MAC=MAC"};
    string mac, schema;
protected:
    friend RemoteSensorServer;
    friend RemoteSensorClient;
    friend Sensor;
    bool seen = false;
public:
    RemoteSensorModule(const char *_mac, const char *_schema = "") : mac(_mac), schema(_schema) {
        for(auto p : requiredFields) { 
            if (strstr(schema.c_str(), p) == NULL)
                schema = string(p) + " " + schema;
        }
        sensors = SchemaParser::parseSchema(schema);
    }
    //RemoteSensorArray(vector<Sensor> &v) { for(auto p : v) { sensors.push_back(&p);}}
    RemoteSensorModule(vector<Sensor *> &v) { for(auto p : v) { sensors.push_back(p); }}
    void addSensor(Sensor *p) { sensors.push_back(p); } 
    float read(const char *key = "RESULT") { return 0; }
    float read(const string &k) { return this->read(k.c_str()); }
    //string makeAllSchema(); 
    //string makeAllResults();
    //void parseAllResults(const string &s);

    bool updateReady = false;
    bool updated() { 
        if (updateReady) { 
            updateReady = false;
            return true;
        }
        return false;
    }
    Sensor *findByName(const char *n) { 
        for(auto i : sensors) { 
            if (i->name == n) 
                return i;
        }
        return NULL;
    }
    Sensor *findByName(const string &s) { return findByName(s.c_str()); }
    string makeAllSchema() { 
        string r;
        for(auto i : sensors) { 
            r += i->name + "=" + i->makeSchema() + " ";
        }
        return r;
    }
    void beginClient() { 
        for(auto i : sensors)  
            i->begin();
    }
    string makeAllResults() { 
        string r;
        findByName(specialWords.MAC)->result = this->mac;
        findByName(specialWords.SCHASH)->result = makeHash();
        for(auto i : sensors) { 
            i->result = i->makeReport();
            r += i->name + "=" + i->result + " ";
        }
        return r;
    }

    string makeHash() {
        string sch = makeAllSchema(); 
        string toHash = "";
        auto words = split(sch, ' ');
        for (auto w : words) { 
            if (w.find(specialWords.SCHASH + "=") != 0)
                toHash += w;
        }
        uint32_t hash = crc32.calc((uint8_t const *)toHash.c_str(), toHash.length());
        return sfmt("%08x", hash);
    }

    void parseAllResults(const string &line) {
        LineMap tokens = parseLine(line);
        for(auto t : tokens) { 
            for(auto s : sensors) { 
                if (s->name == t.first && s->isOutput == false) { 
                    s->result = t.second;
                    updateReady = true;
                    break;
                }
            }
        }
    }

    void parseAllSetValues(const string &s) {
        vector<Sensor *> rval; 
        for (auto w : split(s, ' ')) { 
            string name = w.substr(0, w.find("="));
            string v = w.substr(w.find("=") + 1); 
            for(auto i : sensors) { 
                if (name == i->name && i->isOutput) { 
                    i->setValue(v);
                    break;
                }
           }        
        } 
    }

    string makeAllSetValues() {
        string r;
#if 0 
        // TODO set values not really working, need to clean up 
        for(auto i : sensors) { 
            if (i->isOutput) {
                r += i->name + "=" + i->result + " ";
            }
        }
#endif
        return r;
    }

    void debugPrint() { 
        for(auto ci : sensors) { 
            printf("%s: %s\n", ci->name.c_str(), ci->result.c_str());
        }
    }
};

class SensorWrapper {
private:
    RemoteSensorModule *parent;
    string key;
public: 
    SensorWrapper(RemoteSensorModule *_parent, const string &_key) : parent(_parent), key(_key) {}  
    float read() { return parent->read(key); }  
};

Sensor::Sensor(RemoteSensorModule *parent /*= NULL*/, std::string n /*= ""*/) : name(n) {
    if (parent) parent->addSensor(this);
}

class SensorSchemaHash : public Sensor { 
public:
    SensorSchemaHash(RemoteSensorModule *p = NULL) : Sensor(p, "SCHASH") { result = "NO_HASH"; }
    void begin() {}
    string makeSchema() { return "SCHASH"; }
    string makeReport() { return result; }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorSchemaHash::reg([](const string &s)->Sensor * { 
    return s == "SCHASH" ? new SensorSchemaHash(NULL) : NULL; 
});

class SensorMillis : public Sensor { 
    public:
        SensorMillis(RemoteSensorModule *p) : Sensor(p, "MILLIS") {}
        void begin() {}
        string makeSchema() { return "MILLIS"; }
        string makeReport() { return sfmt("%d", millis()); }
        static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorMillis::reg([](const string &s)->Sensor * { 
    return s == "MILLIS" ? new SensorMillis(NULL) : NULL; 
});
    
class SensorMAC : public Sensor { 
public:
    SensorMAC(RemoteSensorModule *p) : Sensor(p, "MAC") { result = "NO_MAC"; }
    string makeSchema() { return "MAC"; }
    string makeReport() { return result; }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorMAC::reg([](const string &s)->Sensor * { 
    return s == "MAC" ? new SensorMAC(NULL) : NULL; 
});
    
class SensorGit : public Sensor { 
public:
    SensorGit(RemoteSensorModule *p) : Sensor(p, "GIT") {}
    string makeSchema() { return "GIT"; }
    string makeReport() { return string(GIT_VERSION); }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorGit::reg([](const string &s)->Sensor * { 
    return s == "GIT" ? new SensorGit(NULL) : NULL; 
});
    
class SensorADC : public Sensor { 
    int pin;
    float scale;
public:
    SensorADC(RemoteSensorModule *p, const char *n, int _pin, float s = 1.0) : Sensor(p, n), pin(_pin), scale(s) {}    
    string makeSchema() { return sfmt("ADC%d*%f", pin, scale); }
    string makeReport() { return sfmt("%f", avgAnalogRead(pin, 1024) * scale); }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorADC::reg([](const string &s)->Sensor * { 
    float pin, scale;
    if (sscanf(s.c_str(), "ADC%f*%f", &pin, &scale) == 2) 
        return new SensorADC(NULL, "", pin, scale);
    return NULL; 
});

class SensorDHT : public Sensor { 
    DHT dht;
    int pin;
public:
    SensorDHT(RemoteSensorModule *p, const char *n, int _pin) : Sensor(p, n), pin(_pin), dht(_pin, DHT22) {}    
    void begin() override { 
        OUT("DHT.begin()"); 
        pinMode(pin, INPUT_PULLUP); 
        dht.begin(); 
    }
    string makeSchema() override { return sfmt("DHT%d", pin); }
    string makeReport() override {
        float t, h;
        for(int retries = 0; retries < 5; retries++) { 
            t = dht.readTemperature();
            h = dht.readHumidity();
            if (!isnan(t) && !isnan(h))
                break;
            wdtReset();
            delay(1000);
        } 
        return sfmt("%.2f,%.2f", t, h);
    }
    static SchemaParser::RegisterClass reg;
};

SchemaParser::RegisterClass SensorDHT::reg([](const string &s)->Sensor * { 
    int pin;
    if (sscanf(s.c_str(), "DHT%d", &pin) == 1) 
        return new SensorDHT(NULL, "", pin);
    return NULL; 
});

class SensorInput : public Sensor { 
    int pin, mode;
public:
    SensorInput(RemoteSensorModule *p, const char *n, int pi, int m) : Sensor(p, n), pin(pi), mode(m) {}    
    void begin() { pinMode(pin, mode); }
    string makeSchema() {
        const char *m = "";
        if (mode == INPUT_PULLUP) m = "PU";
        if (mode == INPUT_PULLDOWN) m = "PD";  
        return sfmt("INPUT%d%s", pin, m); 
    }
    string makeReport() { return sfmt("%d", digitalRead(pin)); }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorInput::reg([](const string &s)->Sensor * { 
    int pin, mode = INPUT;
    if (sscanf(s.c_str(), "INPUT%d", &pin) == 1) {
        if (strstr(s.c_str(), "PU") != NULL) mode = INPUT_PULLUP;
        if (strstr(s.c_str(), "PD") != NULL) mode = INPUT_PULLDOWN;
        return new SensorInput(NULL, "", pin, mode);
    }
    return NULL; 
});

class SensorOutput : public Sensor { 
    int pin, mode;
public:
    SensorOutput(RemoteSensorModule *p, const char *n, int pi, int m) : Sensor(p, n), pin(pi), mode(m) {
        isOutput = true;
    }    
    void begin() override { 
        setValue(sfmt("%d", mode)); 
    }
    string makeSchema() { return sfmt("OUTPUT%d,%d", pin, mode); }
    string makeReport() { return sfmt("%d", digitalRead(pin)); }
    void setValue(const string &s) { 
        sscanf(s.c_str(), "%d", &mode);
        OUT("SensorOutput pin %d => %d", pin, mode);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, mode);
        result = s;
    }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorOutput::reg([](const string &s)->Sensor * { 
    int pin, mode;
    if (sscanf(s.c_str(), "OUTPUT%d,%d", &pin, &mode) == 2) {
        return new SensorOutput(NULL, "", pin, mode);
    }
    return NULL; 
});

class SensorVariable : public Sensor { 
    string defaultValue;
public:
    string value;
    SensorVariable(RemoteSensorModule *parent, const char *n, const char *v) : Sensor(parent, n), defaultValue(v) {
        isOutput = true;
        value = result = defaultValue;
        setValue(value);
    }    
    string makeSchema() { return sfmt("VAR,%s", defaultValue.c_str()); }
    string makeReport() { return value; }
    void setValue(const string &s) { 
        value = s;
        //OUT("setting variable %s to %s", name.c_str(), value.c_str());
    }
    static SchemaParser::RegisterClass reg;
};
SchemaParser::RegisterClass SensorVariable::reg([](const string &s)->Sensor * { 
    char txt[256];
    if (sscanf(s.c_str(), "VAR,%s", txt) == 1) {
        return new SensorVariable(NULL, "", txt);
    }
    return NULL; 
});


class RemoteSensorServer : public RemoteSensorProtocol { 
    ReliableStreamESPNow fakeEspNow = ReliableStreamESPNow("SN", true);
    vector<RemoteSensorModule *> modules;
    int countSeen() { 
        int rval = 0;
        for(auto p : modules) { 
            if (p->seen) rval++;
        }
        return rval;
    }
    uint32_t lastReportMs = 0, nextSleepTimeMs = 0; // todo avoid rollover by tracking start of sleep period, not end of sleep period
    int serverSleepSeconds = 300;
    int serverSleepLinger = 60;
public: 
    RemoteSensorServer(vector<RemoteSensorModule *> m) : modules(m) {}    
    void onReceive(const string &s) {
        if (s.find(specialWords.ACK) == 0) 
            return;
        string incomingMac = "", incomingHash = "";
        std::map<string, string> in = parseLine(s);
        incomingMac = in[specialWords.MAC];
        incomingHash = in[specialWords.SCHASH];
        if (in.find(specialWords.SERVER) != in.end()) return;
        OUT("%09.4f server <<<< %s", millis() / 1000.0, s.c_str());

        bool packetHandled = false;
        for(auto p : modules) { 
            if (p->mac == incomingMac) {
                packetHandled = true;
                string hash = p->makeHash();

                if (in[specialWords.SCHASH] != hash) { 
                    string out = "SERVER=1 MAC=" + p->mac + " NEWSCHEMA=1 " + p->makeAllSchema();
                    write(out);
                    return;
                }

                for(auto w : split(s, ' ')) { 
                    string name = w.substr(0, w.find("="));
                    string val = w.substr(w.find("=") + 1);
                    if (name == "SCHASH") {
                        if (hash != val) { 
                            string out = "SERVER=1 MAC=" + p->mac + " NEWSCHEMA=1 " + p->makeAllSchema();
                            write(out);
                            return;
                        }
                    } else if (0) { 
                        string out = "SERVER=1 MAC=" + p->mac + " UPDATENOW=1 ...";
                        write(out);
                    } else if (name == "ENDLINE") {
                        break; 
                    }
                }
                if (hash == incomingHash) {
                    if (countSeen() == 0) {
                        nextSleepTimeMs = millis() + serverSleepSeconds * 1000;
                    } 
                    p->seen = true;
                    lastReportMs = millis();
                    p->parseAllResults(s);
                    int moduleSleepSec = (nextSleepTimeMs - millis()) / 1000;
                    if (moduleSleepSec > serverSleepSeconds)
                        moduleSleepSec = 5;
                    string out = "SERVER=1 MAC=" + p->mac + " SCHASH=" + hash + " SLEEP=" 
                        + sfmt("%d ", moduleSleepSec) + p->makeAllSetValues();
                    write(out);
                } else {
                    OUT("%s != %s", incomingHash.c_str(), hash.c_str());
                }
            }
        }
        if (!packetHandled && incomingMac != "") { 
            OUT("Unknown MAC: %s", s.c_str());
            for(auto p : modules) { 
                if (p->mac == "auto") {
                    p->mac = incomingMac;
                    string schema = p->makeAllSchema();
                    OUT("Auto assigning incoming mac %s to sensor %s", p->mac.c_str(), schema.c_str());
                    break;
                }
            }
        }
    }

    void write(const string &s) { 
        OUT("%09.4f server >>>> %s", millis() / 1000.0, s.c_str());
        fakeEspNow.write(s);
    }

    void begin() { 
        // set up ESPNOW, register this->onReceive() listener 
    }
    void run() {
        string in = fakeEspNow.read();
        if (in != "") 
            onReceive(in);
        if (j.secTick(5)) { 
            OUT("Seen %d/%d modules, last traffic %d sec ago", countSeen(), modules.size(), (millis() - lastReportMs) / 1000);
        }
        if (countSeen() > 0 && (nextSleepTimeMs - millis()) / 1000 > serverSleepSeconds) {
            // missing some sensors but the entire sleep period has elapsed?  Just reset to next sleep Period 
            nextSleepTimeMs = millis() + serverSleepSeconds * 1000;
        }

        if (countSeen() > 0 && countSeen() == modules.size() && (millis() - lastReportMs) > serverSleepLinger * 1000) {
            int sleepSec = (nextSleepTimeMs - millis()) / 1000;
            if (sleepSec > serverSleepSeconds)
                sleepSec = 0;
            for(auto p : modules) p->seen = false;
            OUT("All %d modules accounted for, time to sleep %d sec", modules.size(), sleepSec);
        }
    }
};

class RemoteSensorClient : public RemoteSensorProtocol { 
    string mac = getMacAddress().c_str();
    ReliableStreamESPNow fakeEspNow = ReliableStreamESPNow("SN", true);
    RemoteSensorModule *array = NULL;
    static SPIFFSVariable<int> lastChannel;
    static SPIFFSVariable<string> lastSchema;
    uint32_t inhibitStartMs, inhibitMs = 0;
    bool deepSleep = true;
public:
    void csimOverrideMac(const string &s) { 
        mac = s;
        if (array != NULL) delete array;
        array = NULL;
        string sch = lastSchema;
        init(sch);
        deepSleep = false;
    } 
    RemoteSensorClient() { 
        string s = lastSchema;
        espNowMux.defaultChannel = lastChannel; 
        init(s);
    }
    void init(const string &schema) { 
        if (array != NULL) delete array;
        array = new RemoteSensorModule(mac.c_str(), schema.c_str());
        array->beginClient();
        lastReceive = millis();
    }
    void updateFirmware() {
        // TODO
    }
    bool updateFirmwareNow = false, updateSchemaNow = false;
    uint32_t lastReceive = 0;
    void onReceive(const string &s) { 
        LineMap in = parseLine(s);
        if (in.contains(specialWords.ACK)) return;
        if (in.contains(specialWords.SERVER) == false) return;
        
        string schash, newSchema;
        bool updatingSchema = false;   
        int sleepTime = -1;     
        for(auto w : split(s, ' ')) { 
            if (updatingSchema) {
                newSchema += w + " ";
            } else { 
                string name = w.substr(0, w.find("="));
                string val = w.substr(w.find("=") + 1);
                if (name == "MAC") {
                    if (val != mac) return;
                    lastChannel = espNowMux.defaultChannel;
                    lastReceive = millis();
                }
                else if (name == "NEWSCHEMA") updatingSchema = true;
                else if (name == "UPDATENOW") updateFirmware();
                else if (name == "SLEEP") sscanf(val.c_str(), "%d", &sleepTime);
            }
        }
        OUT("%09.4f client <<<< %s", millis() / 1000.0, s.c_str());
        if (updatingSchema) { 
            OUT("Got new schema: %s", newSchema.c_str());
            init(newSchema);
            lastSchema = newSchema;
            string out = array->makeAllResults();
            write(out);
            return;
        }
        if (array)
            array->parseAllSetValues(s);
        // TODO:  Write schema and shit to SPIFF
        if (sleepTime > 0) { 
            if (deepSleep) { 
                OUT("%09.4f: Sleeping %d seconds...", millis() / 1000.0, sleepTime);
                WiFi.disconnect(true);  // Disconnect from the network
                WiFi.mode(WIFI_OFF);    // Switch WiFi off
                int rc = esp_sleep_enable_timer_wakeup(1000000LL * sleepTime);
                Serial.flush();
                //esp_light_sleep_start();                                                                 
                esp_deep_sleep_start();
                //delay(1000 * sleepTime);                                                                 
                ESP.restart();                                 
            } else { 
                inhibitStartMs = millis();
                inhibitMs = sleepTime * 1000;
            } 
        }
    }
    void write(const string &s) { 
        OUT("%09.4f client >>>> %s", millis() / 1000.0, s.c_str());
        fakeEspNow.write(s);
    }
    void run() {
        string in = fakeEspNow.read();
        if (in != "") 
            onReceive(in);
            
        if (inhibitMs > 0) { 
            if (millis() - inhibitStartMs > inhibitMs) { 
                inhibitMs = 0;
                lastReceive = millis();
            }
        } else { 
            if (array != NULL && (j.once() || j.secTick(.2))) { 
                string out = array->makeAllResults() + "ENDLINE=1 ";
                write(out);
            }
            // channel hopping
            if (millis() - lastReceive > 10000) {
                espNowMux.defaultChannel = (espNowMux.defaultChannel + 1) % 14;
                espNowMux.stop();
                espNowMux.firstInit = true;
                lastReceive = millis();
            }
        }
    }
};

SPIFFSVariable<int> RemoteSensorClient::lastChannel 
    = SPIFFSVariable<int>("/sensorNetwork_lastChannel2", 1);
SPIFFSVariable<string> RemoteSensorClient::lastSchema 
    = SPIFFSVariable<string>("/sensorNetwork_lastSchema", "MAC=MAC SKHASH=SKHASH GIT=GIT MILLIS=MILLIS");


//static SchemaList::Register();
#endif //__SENSORNETWORKESPNOW_H_