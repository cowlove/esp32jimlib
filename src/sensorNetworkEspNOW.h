#ifndef __SENSORNETWORKESPNOW_H_
#define __SENSORNETWORKESPNOW_H_
#include "espNowMux.h"
#include "reliableStream.h"
#include "Arduino_CRC32.h" 
#ifndef CSIM
#include "DHT.h"
#endif

#include "jimlib.h"

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
    int pin;
    int retries = 0;
public:
    DHT dht;
    SensorDHT(RemoteSensorModule *p, const char *n, int _pin) : Sensor(p, n), pin(_pin), dht(_pin, DHT22) {}    
    void begin() override { 
        printf("%09.3f DHT begin()\n", millis()/1000.0);
        pinMode(pin, INPUT_PULLUP); 
        dht.begin(); 
    }
    string makeSchema() override { return sfmt("DHT%d", pin); }
    string makeReport() override {
        float t = NAN, h = NAN;
        for(int r = 0; r < 15; r++) {
            while(millis() < 750) delay(10); // HACK - DHT has trouble if read before power is stable for about 650ms
            h = dht.readHumidity();
            t = dht.readTemperature();
            if (!isnan(t) && !isnan(h)) {
                break;
            } 
            retries++;
            printf("%09.3f DHT read failure t:%.2f h:%.2f, retry #%d, total retries %d\n", millis()/1000.0, t, h, r, retries);
            wdtReset();
            delay(200);
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
        printf("%09.3f Setting pin %d => %d\n", millis()/1000.0, pin, mode);
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
    SPIFFSVariable<vector<string>> spiffResultLog = SPIFFSVariable<vector<string>>("/RemoteSensorSever.Log", {});
    
    uint32_t lastReportMs = 0, nextSleepTimeMs = 0; 
    // time remaining in sleep after being interrupted by pauseSleep();
    SPIFFSVariable<int> spiffsResumeSleepMs = SPIFFSVariable<int>("/RemSenSrv_srs", 0);
public: 
    int countSeen() { 
        int rval = 0;
        for(auto p : modules) { 
            if (p->seen) rval++;
        }
        return rval;
    }
    float lastTrafficSec() { 
        return (millis() - lastReportMs) / 1000.0;
    }
    int serverSleepSeconds = 90;
    int serverSleepLinger = 10;
    RemoteSensorServer(vector<RemoteSensorModule *> m) : modules(m) {
        // BAD hack to resume values after deep sleep - replay log of last received valid data lines
        vector<string> v = spiffResultLog;
        for(auto s : v) {
            std::map<string, string> in = parseLine(s);
            for(auto p : modules) { 
                if (p->mac == "auto" && in.find(specialWords.MAC) != in.end())
                    p->mac = in[specialWords.MAC];
                if (p->mac == in[specialWords.MAC]) {
                    string hash = p->makeHash();
                    if (in[specialWords.SCHASH] == hash) { 
                        p->parseAllResults(s);
                    }
                }
            }
        }
        if (spiffsResumeSleepMs > 0) {
            for(auto p : modules) 
                p->seen = true;
            nextSleepTimeMs = millis() + spiffsResumeSleepMs;
            spiffsResumeSleepMs = 0;
        }
        lastReportMs = 0;
    }    
    void prepareSleep(int ms) {
        int sleepLeftMs = ((int)nextSleepTimeMs) - millis() - ms;
        if (sleepLeftMs < 10 * 1000) { 
            // close to, or after our expected wake time, don't resumed the period
            for(auto p : modules) p->seen = false;
            spiffsResumeSleepMs = 0;
        } else { 
            // resume this planned wait period after this coming short sleep
            spiffsResumeSleepMs = sleepLeftMs;
        }
    }
    void onReceive(const string &s) {
        if (s.find(specialWords.ACK) == 0) 
            return;
        string incomingMac = "", incomingHash = "";
        std::map<string, string> in = parseLine(s);
        incomingMac = in[specialWords.MAC];
        incomingHash = in[specialWords.SCHASH];
        if (in.find(specialWords.SERVER) != in.end()) return;
        printf("%09.3f server <<<< %s\n", millis() / 1000.0, s.c_str());

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
                    if (nextSleepTimeMs - millis() < 10000 || nextSleepTimeMs - millis() > serverSleepSeconds * 1000) {
                        for(auto p : modules) p->seen = false;
                    } 
                    if (countSeen() == 0) {
                        nextSleepTimeMs = millis() + serverSleepSeconds * 1000;
                    } 
                    p->seen = true;
                    lastReportMs = millis();
                    p->parseAllResults(s);
                    int moduleSleepSec = (nextSleepTimeMs - millis()) / 1000;
                    if (moduleSleepSec > serverSleepSeconds)
                        moduleSleepSec = serverSleepSeconds;;
                    string out = "SERVER=1 MAC=" + p->mac + " SCHASH=" + hash + " SLEEP=" 
                        + sfmt("%d ", moduleSleepSec) + p->makeAllSetValues();
                    write(out);
                    // HACK - keep rolling log of good data lines to replay after a deep sleep
                    vector<string> log = spiffResultLog;
                    log.push_back(s);
                    if (log.size() > modules.size() * 3)
                        log.erase(log.begin());
                    spiffResultLog = log;
                } else {
                    printf("HASH: %s != %s\n", incomingHash.c_str(), hash.c_str());
                }
            }
        }
        if (!packetHandled && incomingMac != "") { 
            printf("Unknown MAC: %s\n", s.c_str());
            for(auto p : modules) { 
                if (p->mac == "auto") {
                    p->mac = incomingMac;
                    string schema = p->makeAllSchema();
                    printf("Auto assigning incoming mac %s to sensor %s\n", p->mac.c_str(), schema.c_str());
                    onReceive(s);
                    break;
                }
            }
        }
    }

    void write(const string &s) { 
        printf("%09.3f server >>>> %s\n", millis() / 1000.0, s.c_str());
        fakeEspNow.write(s);
    }

    void begin() { 
        // set up ESPNOW, register this->onReceive() listener 
    }
    void run() {
        string in = fakeEspNow.read();
        if (in != "") 
            onReceive(in);
        static HzTimer timer(.1);
        if (timer.tick()) { 
            //printf("%09.3f Seen %d/%d modules, last traffic %d sec ago\n",
            // millis() / 1000.0, countSeen(), (int)modules.size(), (int)(millis() - lastReportMs) / 1000);
        }
        if (countSeen() > 0 && (nextSleepTimeMs - millis()) / 1000 > serverSleepSeconds) {
            // missing some sensors but the entire sleep period has elapsed?  Just reset to next sleep Period 
            nextSleepTimeMs = millis() + serverSleepSeconds * 1000;
            lastReportMs = millis();                
        }
    }
    float getSleepRequest() { 
        if (countSeen() == modules.size() && (millis() - lastReportMs) > serverSleepLinger * 1000) {
            float sleepSec = (nextSleepTimeMs - millis()) / 1000.0;
            if (sleepSec > serverSleepSeconds)
                sleepSec = -1;
            if (sleepSec < 20)
                sleepSec = -1;
            //printf("All %d modules accounted for, OK to sleep %d sec\n", (int)modules.size(), sleepSec);
            return sleepSec;  
        } 
        return -1;
    }
};

class RemoteSensorClient : public RemoteSensorProtocol { 
    string mac = getMacAddress().c_str();
    ReliableStreamESPNow fakeEspNow = ReliableStreamESPNow("SN", true);
    RemoteSensorModule *array = NULL;
    SPIFFSVariable<int> *lastChannel = NULL, *sleepRemainingMs;
    SPIFFSVariable<string> *lastSchema = NULL;
    uint32_t inhibitStartMs, inhibitMs = 0;
    bool deepSleep = true;
public:
    bool channelHop = false;
    void csimOverrideMac(const string &s) { 
        mac = s;
        init();
        deepSleep = false;
    } 
    Sensor *findByName(const char *n) { 
        return array == NULL ? NULL : array->findByName(n);
    }
    RemoteSensorClient() { 
        init();
    }
    void init(const string &schema = "") { 
        if (array != NULL) {
            delete array;
            delete lastChannel;
            delete lastSchema;
            delete sleepRemainingMs;
        }
        lastChannel = new SPIFFSVariable<int>(("/sn_lc_" + mac).c_str(), 1);
        lastSchema = new SPIFFSVariable<string>(("/sn_ls_" + mac).c_str(), "MAC=MAC SKHASH=SKHASH GIT=GIT MILLIS=MILLIS");
        sleepRemainingMs = new SPIFFSVariable<int>(("/sn_sr_" + mac).c_str(), 0);
        if (schema != "")
            *lastSchema = schema;
        string s = *lastSchema;
        array = new RemoteSensorModule(mac.c_str(), s.c_str());
        array->beginClient();
        espNowMux.defaultChannel = *lastChannel; 
        if (*sleepRemainingMs > 0) { // look if setPartialDeepSleep() was called, resume sleeping
            inhibitStartMs = millis();
            inhibitMs = (int)*sleepRemainingMs;
            *sleepRemainingMs = 0;
        } else { 
            inhibitMs = inhibitStartMs = 0; // otherwise start cycle right now
        }
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
                    *lastChannel = espNowMux.defaultChannel;
                    lastReceive = millis();
                }
                else if (name == "NEWSCHEMA") updatingSchema = true;
                else if (name == "UPDATENOW") updateFirmware();
                else if (name == "SLEEP") sscanf(val.c_str(), "%d", &sleepTime);
            }
        }
        printf("%09.3f client <<<< %s\n", millis() / 1000.0, s.c_str());
        if (updatingSchema) { 
            printf("Got new schema: %s\n", newSchema.c_str());
            init(newSchema);
            string out = array->makeAllResults();
            write(out);
            return;
        }
        if (array)
            array->parseAllSetValues(s);
        // TODO:  Write schema and shit to SPIFF
        if (sleepTime > 0) { 
            if (deepSleep) { 
                printf("%09.3f: Sleeping %d seconds...\n", millis() / 1000.0, sleepTime);
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
        printf("%09.3f client >>>> %s\n", millis() / 1000.0, s.c_str());
        fakeEspNow.write(s);
    }
    void run() {
        string in = fakeEspNow.read();
        if (in != "") 
            onReceive(in);
            
        if (inhibitMs > 0) { 
            if (millis() - inhibitStartMs > inhibitMs) { 
                init();
            }
        } else { 
            static HzTimer timer(.6, true);
            if (array != NULL && timer.secTick(1.0)) { 
                string out = array->makeAllResults() + "ENDLINE=1 ";
                write(out);
            }
            // channel hopping
            if (channelHop == true && millis() - lastReceive > 10000) {
                espNowMux.defaultChannel = (espNowMux.defaultChannel + 1) % 14;
                espNowMux.stop();
                espNowMux.firstInit = true;
                lastReceive = millis();
            }
        }
    }
    void setPartialDeepSleep(uint64_t usec) {
        int remainMs = inhibitMs - (millis() - inhibitStartMs) - usec / 1000;
        remainMs = max(0, remainMs);
        *sleepRemainingMs = remainMs;
    }
};



//static SchemaList::Register();
#endif //__SENSORNETWORKESPNOW_H_