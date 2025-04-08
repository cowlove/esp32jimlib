#include "sensorNetworkEspNOW.h"

void RemoteSensorServer::checkInit() {
    if (initialized == true) return;
    initialized = true;
    if (getResetReason(0) != 5) {
        lastSynchTs.set(synchPeriodMin * 60 * 1000);
        sensorsCompleteOnSleep = false;
    }
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
    for(auto p : modules) 
        p->seen = sensorsCompleteOnSleep;
    sensorsCompleteOnSleep = false;
    //if (spiffsResumeSleepMs > 0) {
    //    for(auto p : modules) 
    //        p->seen = true;
    //    nextSleepTimeMs = millis() + spiffsResumeSleepMs;
    //    spiffsResumeSleepMs = 0;
    //}
}
int RemoteSensorServer::countSeen() { 
    int rval = 0;
    for(auto p : modules) { 
        if (p->seen) rval++;
    }
    return rval;
}
    float RemoteSensorServer::lastTrafficSec() { 
        return (lastSynchTs.elapsed()) / 1000.0;
    }
 
    RemoteSensorServer::RemoteSensorServer(vector<RemoteSensorModule *> m) : modules(m) {
        // BAD hack to resume values after deep sleep - replay log of last received valid data lines
    }    
    void RemoteSensorServer::prepareSleep(int ms) {
        lastSynchTs.prepareSleep(ms);
        sensorsCompleteOnSleep = (countSeen() == modules.size());
    }
    void RemoteSensorServer::onReceive(const string &s) {
        checkInit();
        if (s.find(specialWords.ACK) == 0) 
            return;
        string incomingMac = "", incomingHash = "";
        std::map<string, string> in = parseLine(s);
        incomingMac = in[specialWords.MAC];
        incomingHash = in[specialWords.SCHASH];
        if (in.find(specialWords.SERVER) != in.end()) return;

        float late = lastLastSynchTs.elapsed() / 1000.0 - synchPeriodMin * 60;
        printf("%09.3f %09.3f server <<<< %s (%.3fs late)\n", 
            deepsleepMs.millis() / 1000.0, millis() / 1000.0, s.c_str(), 
            late);

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
                    if (countSeen() == 0) { // first sensor? reset synch period 
                        lastLastSynchTs.set(lastSynchTs.millis());
                        lastSynchTs.reset();
                    } 
                    p->seen = true;
                    lastReportTs = millis();
                    p->parseAllResults(s);
                    int moduleSleepSec = ((synchPeriodMin * 60 * 1000) - lastSynchTs.elapsed()) / 1000; 
                    //int moduleSleepSec = (nextSleepTimeMs - millis()) / 1000;
                    //if (moduleSleepSec > serverSleepSeconds)
                    //    moduleSleepSec = serverSleepSeconds;;
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

    void RemoteSensorServer::write(const string &s) { 
        printf("%09.3f %09.3f server >>>> %s\n", deepsleepMs.millis()/1000.0, millis() / 1000.0, s.c_str());
        fakeEspNow.write(s);
    }

//    void RemoteSensorServer::begin() { 
//        // set up ESPNOW, register this->onReceive() listener 
//    }
    void RemoteSensorServer::run() {
        checkInit();
        if (lastSynchTs.elapsed() > ((synchPeriodMin * 60) - earlyWakeupSec) * 1000) { 
            lastLastSynchTs.set(lastSynchTs.millis()); // use for measuring/debugging
            lastSynchTs.reset();
            for(auto p : modules) p->seen = false;  // resets countSeen() below
        } 
        string in = fakeEspNow.read();
        if (in != "") 
            onReceive(in);
        static HzTimer timer(.1);
        if (timer.tick()) { 
            //printf("%09.3f Seen %d/%d modules, last traffic %d sec ago\n",
            // millis() / 1000.0, countSeen(), (int)modules.size(), (int)(millis() - lastReportMs) / 1000);
        }
        //if (countSeen() > 0 && (nextSleepTimeMs - millis()) / 1000 > serverSleepSeconds) {
        //    // missing some sensors but the entire sleep period has elapsed?  Just reset to next sleep Period 
        //    nextSleepTimeMs = millis() + serverSleepSeconds * 1000;
        //    lastReportMs = millis();                
        //}
    }

    float RemoteSensorServer::getSleepRequest() { // rename to getSleepRequestSec()
        checkInit();
        float sleepSec = -1;
        float newSleepSec = synchPeriodMin * 60 - earlyWakeupSec - lastSynchTs.elapsed() / 1000.0;  
        float lastSynchAgeMin = lastSynchTs.elapsed() / 1000.0 / 60.0;
        
        if (0) { 
            printf("%09.3f %09.3f lastSynchTs %d, lastSynchAge %.2f sleepSec %.2f countSeen %d modules %d\n", 
                millis()/1000.0, deepsleepMs.millis()/1000.0, (int)lastSynchTs.elapsed(), lastSynchAgeMin, newSleepSec, countSeen(), (int)modules.size());
        }
        if (countSeen() == 0 && clientTimeoutZeroTrafficMin > 0 
            && lastSynchAgeMin > synchPeriodMin + clientTimeoutZeroTrafficMin)
            sleepSec = newSleepSec;
        if (countSeen() > 0 && countSeen() < modules.size() && lastSynchAgeMin > clientTimeoutSec / 60)
            sleepSec = newSleepSec;
        if (countSeen() == modules.size() && lastSynchAgeMin > lingerSec / 60)
            sleepSec = newSleepSec;
        
        if (sleepSec < 0) sleepSec = -1;
        return sleepSec;  
    }

    void RemoteSensorClient::csimOverrideMac(const string &s) { 
        mac = s;
        init();
        deepSleep = false;
    } 
    Sensor *RemoteSensorClient::findByName(const char *n) { 
        return array == NULL ? NULL : array->findByName(n);
    }
    RemoteSensorClient::RemoteSensorClient() { 
        init();
    }
    void RemoteSensorClient::init(const string &schema/*=""*/) { 
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
    void RemoteSensorClient::updateFirmware() {
        // TODO
    }
    void RemoteSensorClient::onReceive(const string &s) { 
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
    void RemoteSensorClient::write(const string &s) { 
        printf("%09.3f client >>>> %s\n", millis() / 1000.0, s.c_str());
        fakeEspNow.write(s);
    }
    void RemoteSensorClient::run() {
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
    void RemoteSensorClient::setPartialDeepSleep(uint64_t usec) {
        int remainMs = inhibitMs - (millis() - inhibitStartMs) - usec / 1000;
        remainMs = max(0, remainMs);
        *sleepRemainingMs = remainMs;
    }
    
    Sensor::Sensor(RemoteSensorModule *parent /*= NULL*/, std::string n /*= ""*/) : name(n) {
        if (parent) parent->addSensor(this);
    }
    vector<SchemaParser::ParserFunc> &SchemaParser::parserList() { 
        static vector<SchemaParser::ParserFunc> *onFirstUse = new vector<SchemaParser::ParserFunc>();
        return *onFirstUse;
    }
    
