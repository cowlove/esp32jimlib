#ifndef __SENSORNETWORKESPNOW_H_
#define __SENSORNETWORKESPNOW_H_
#include "jimlib.h"
#include "espNowMux.h"
#include "reliableStream.h"
#include "Arduino_CRC32.h" 
#include "serialLog.h"
#include "Adafruit_HX711.h"

#ifndef CSIM
#include "DHT.h"
#endif


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
    uint32_t updateTimeMs = 0xf000000;
    Sensor(RemoteSensorModule *p = NULL, std::string n = "");
    virtual void begin() {}
    virtual string makeSchema() = 0;
    virtual string makeReport() = 0;
    virtual void setValue(const string &s) {}
    string name = "";
    string result = "";
    float asFloat() const { return strtof(result.c_str(), NULL); }
    int asInt() const { return strtol(result.c_str(), NULL, 10); }
    string asString() const { return result; }
    uint32_t getAgeMs() const { return millis() - updateTimeMs; }
};

class SchemaParser : protected RemoteSensorProtocol { 
public:
    typedef Sensor *(*ParserFunc)(const string &);
    static vector<ParserFunc> &parserList();

    static vector<Sensor *> parseSchema(const string &s) {
        vector<Sensor *> rval; 
        for (auto w : split(s, ' ')) { 
            string name = w.substr(0, w.find("="));
            string sch = w.substr(w.find("=") + 1); 
            for(auto i : parserList()) { 
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
        RegisterClass(ParserFunc a) { parserList().push_back(a); }
    };
};

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
        r.reserve(256);
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
        r.reserve(256);
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
                    s->updateTimeMs = millis();
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


class SensorSchemaHash : public Sensor { 
public:
    SensorSchemaHash(RemoteSensorModule *p = NULL) : Sensor(p, "SCHASH") { result = "NO_HASH"; }
    void begin() {}
    string makeSchema() { return "SCHASH"; }
    string makeReport() { return result; }
    static SchemaParser::RegisterClass reg;
};
inline SchemaParser::RegisterClass SensorSchemaHash::reg([](const string &s)->Sensor * { 
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
inline SchemaParser::RegisterClass SensorMillis::reg([](const string &s)->Sensor * { 
    return s == "MILLIS" ? new SensorMillis(NULL) : NULL; 
});
    
class SensorMAC : public Sensor { 
public:
    SensorMAC(RemoteSensorModule *p) : Sensor(p, "MAC") { result = "NO_MAC"; }
    string makeSchema() { return "MAC"; }
    string makeReport() { return result; }
    static SchemaParser::RegisterClass reg;
};
inline SchemaParser::RegisterClass SensorMAC::reg([](const string &s)->Sensor * { 
    return s == "MAC" ? new SensorMAC(NULL) : NULL; 
});
    
class SensorGit : public Sensor { 
public:
    SensorGit(RemoteSensorModule *p) : Sensor(p, "GIT") {}
    string makeSchema() { return "GIT"; }
    string makeReport() { return string(GIT_VERSION); }
    static SchemaParser::RegisterClass reg;
};
inline SchemaParser::RegisterClass SensorGit::reg([](const string &s)->Sensor * { 
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
inline SchemaParser::RegisterClass SensorADC::reg([](const string &s)->Sensor * { 
    float pin, scale;
    if (sscanf(s.c_str(), "ADC%f*%f", &pin, &scale) == 2) 
        return new SensorADC(NULL, "", pin, scale);
    return NULL; 
});

class SensorDHT : public Sensor { 
    int pin;
    int retries = 0;
public:
    DHT dht;
    SensorDHT(RemoteSensorModule *p, const char *n, int _pin) : Sensor(p, n), pin(_pin), dht(_pin, DHT22) {}    
    void begin() override { 
        //OUT("DHT begin()\n");
        pinMode(pin, INPUT_PULLUP); 
        dht.begin(); 
    }
    string makeSchema() override { return sfmt("DHT%d", pin); }
    string makeReport() override {
        float t = NAN, h = NAN;
        for(int r = 0; r < 15; r++) {
            //while(millis() < 750) delay(10); // HACK - DHT has trouble if read before power is stable for about 650ms
            h = dht.readHumidity();
            t = dht.readTemperature();
            if (!isnan(t) && !isnan(h)) {
                break;
            } 
            retries++;
            printf("%09.3f DHT read failure t:%.2f h:%.2f, retry #%d, total retries %d\n", millis()/1000.0, t, h, r, retries);
            wdtReset();
            delay(500);
        } 
        return sfmt("%.2f,%.2f,%d", t, h, retries);
    }
    float getTemperature() const { 
        float t = NAN, h = NAN;
        sscanf(result.c_str(), "%f,%f", &t, &h);
        return t;
    }
    float getHumidity() const { 
        float t = NAN, h = NAN;
        sscanf(result.c_str(), "%f,%f", &t, &h);
        return h;
    }
    int getRetries() const { 
        float t = NAN, h = NAN;
        int retries = 0;
        sscanf(result.c_str(), "%f,%f,%d", &t, &h, &retries);
        return retries;
    }
    static SchemaParser::RegisterClass reg;
};

inline SchemaParser::RegisterClass SensorDHT::reg([](const string &s)->Sensor * { 
    int pin;
    if (sscanf(s.c_str(), "DHT%d", &pin) == 1) 
        return new SensorDHT(NULL, "", pin);
    return NULL; 
});

class SensorHX711 : public Sensor { 
    int clk, data;
    int retries = 0;
public:
    Adafruit_HX711 hx;
    SensorHX711(RemoteSensorModule *p, const char *n, int _clk, int _data) : 
        Sensor(p, n), clk(_clk), data(_data), hx(_data, _clk) {}    
    void begin() override { 
        hx.begin(); 
    }
    string makeSchema() override { return sfmt("HX711_%d,%d", clk, data); }
    string makeReport() override {
        float v = 0;
        int reads = 10;
        for(int r = 0; r < reads; r++) {
            v += hx.readChannelRaw();
        } 
        v /= reads;
        return sfmt("%.2f", v);
    }
    float get() const { 
        float t = NAN;
        sscanf(result.c_str(), "%f", &t);
        return t;
    }
    static SchemaParser::RegisterClass reg;
};

inline SchemaParser::RegisterClass SensorHX711::reg([](const string &s)->Sensor * { 
    int clk, data;
    if (sscanf(s.c_str(), "HX711_%d,%d", &clk, &data) == 1) 
        return new SensorHX711(NULL, "", clk, data);
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

inline SchemaParser::RegisterClass SensorInput::reg([](const string &s)->Sensor * { 
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
        //OUT("Setting pin %d => %d\n", pin, mode);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, mode);
        result = s;
    }
    static SchemaParser::RegisterClass reg;
};
inline SchemaParser::RegisterClass SensorOutput::reg([](const string &s)->Sensor * { 
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
    }
    static SchemaParser::RegisterClass reg;
};
inline SchemaParser::RegisterClass SensorVariable::reg([](const string &s)->Sensor * { 
    char txt[256];
    if (sscanf(s.c_str(), "VAR,%s", txt) == 1) {
        return new SensorVariable(NULL, "", txt);
    }
    return NULL; 
});


// Timing/Sleep protocol
//
// Server maintains a reporting timestamp.  Server wakes up or boots slightly before the timestamp,
// and listens for clients.  The server sets the reporting timestamp upon hearing from the first client,
//  After all clients have been heard, it waits a linger time to ensure
//  all clients are sleeping and allows a sleep until earlyWakeupSec before the next synch
// Sequence:
// 1) server boots or wakes with no state and assumes it is near a reporting timestamp, and does not allow
//    sleeps
// 2) Upon receipt of the first timestamp, sets lastSynchTs and begins counting, notifyihg clients
//    to sleep for lastSynchTs + sychPeriodMin, and still does not allow sleeps
// 3) After all clients have been heard and notified of the wakeup time, server waits
//    lingerSec not allowing sleeps, to make sure all clients are accounted for
// 4) If a client is missing, the server optionally allows sleeps after min(lingerSec,clientTimeoutSec)
// 5) If no sleep happens and the server keeps waiting, it will hear the clients begin to report in
//    for the next synch period.   If it is earlier than earlyWakupSec before the synch point,
//    it will put them back to sleep with a short sleep duration.  It is within earlyWakupSec of the
//    synch point, the server will reset lastSynchTs and continue at step 2
// 
//    The main program loop should avoid logging, sleeping, or using the network while the server is 
//    listening for clients.  This can be accomplished by only doing those things while sleep is allowed
//    with getAllowedSleepMin() of about the expected time to accomplish the task 
//   
//    Not every wakeup is necessarily step #1.  The program loop may wakeup the MCU for other reasons.
//    The server registers if all clients are complete in prepareSleep() 
//
//    A few complications exist with a hard boot and a zero lastSynchTs.   This is handled by setting
//    lastSynchTs to millis() - synchPeriodMin and extending a larger clientTimeoutZeroClientsSec to 
//    allow catching the first client.  The first client will reset lastSynchTs and after that 
//    clientTimeoutSec/lingerSec will apply. 

class RemoteSensorServer : public RemoteSensorProtocol { 
    ReliableStreamESPNow fakeEspNow = ReliableStreamESPNow("SN", true);
    vector<RemoteSensorModule *> modules;

    // used to pre-feed and initialize so server boots/wakes with the last values.  cleared on hard boot 
    SPIFFSVariable<vector<string>> spiffResultLog = SPIFFSVariable<vector<string>>("/RemoteSensorSever.Log", {});
    DeepSleepElapsedTimer lastSynchTs = DeepSleepElapsedTimer("/RemSenSrv_ls", 0);
    DeepSleepElapsedTimer lastLastSynchTs = DeepSleepElapsedTimer("/RemSenSrv_lls", 0); // just for measuring/debugging
    SPIFFSVariable<bool> sensorsCompleteOnSleep = SPIFFSVariable<bool>("/RemSenSrv_sCoS", false);
    DeepSleepElapsedTimer lastTrafficTs = DeepSleepElapsedTimer("/RemSenSrv_lts", 0);

    bool initialized = false;
    void checkInit();
    RemoteSensorModule *findByMac(const string &);
public: 
    //int serverSleepSeconds = 90;
    //int serverSleepLinger = 10;
    float synchPeriodMin = 2.0;
    float lingerSec = 5;
    float clientTimeoutSec = 30;
    float clientTimeoutZeroTrafficMin = -1; /* < 0 never timeout for zero clients */
    float earlyWakeupSec = 5;

    int countSeen();
    float lastTrafficSec();
    RemoteSensorServer(vector<RemoteSensorModule *> m);
    void prepareSleep(int ms);
    void onReceive(const string &s);
    void write(const string &s);    
    //void begin();
    void run();
    float getSleepRequest();
};

class RemoteSensorClient : public RemoteSensorProtocol { 
    string mac;
    ReliableStreamESPNow fakeEspNow = ReliableStreamESPNow("SN", true);
    RemoteSensorModule *array = NULL;
    SPIFFSVariable<int> *lastChannel = NULL, *sleepRemainingMs;
    SPIFFSVariable<string> *lastSchema = NULL;
    uint32_t inhibitStartMs, lastReceive = 0, inhibitMs = 0;
    bool allowDeepSleep = true;
    void checkInit();
    HzTimer timer = HzTimer(.6, true);
public:
    RemoteSensorClient();
    bool channelHop = false;
    void csimOverrideMac(const string &s);
    Sensor *findByName(const char *n);
    void init(const string &schema = "");
    void updateFirmware();
    void onReceive(const string &s);
    void write(const string &s);
    void run();
    void prepareSleep(uint32_t ms);
};

//static SchemaList::Register();
#endif //__SENSORNETWORKESPNOW_H_