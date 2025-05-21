#ifdef ESP32
#include "batchWebLogger.h"
#include "serialLog.h"
#include "jimlib.h"

#ifndef CSIM
#include "FS.h"
#include "LittleFS.h"
#ifdef ESP32
#include "HTTPClient.h"
#else
#include "ESP8266HTTPClient.h"
#endif
#endif

#ifndef OUT
static inline void nullOut(const char *f, ...) {}
#define OUT nullOut
#endif

string FileLineLogger::getNextLine(fs::File &f) { 
    string line;
    while(true) { 
        uint8_t c;
        int n = f.read(&c, 1);
        if (n != 1 || c == '\n') break;
        if (c != '\0') line += c;
    }
    return line;
}
FileLineLogger::FileLineLogger(const string &fn) : filename(fn) {}
void FileLineLogger::push_back(const string &s) { 
    lineCount = getLines() + 1;
    fs::File f = LittleFS.open(filename.c_str(), "a");
    f.write((uint8_t*)s.c_str(), s.length());
    f.write('\n');
}
string FileLineLogger::getLine(int lineNumber) { 
    fs::File f = LittleFS.open(filename.c_str(), "r");
    string line;
    while(lineNumber-- >= 0) line = getNextLine(f);
    return line;
}
vector<string> FileLineLogger::getFirstLines(int count) {
    fs::File f = LittleFS.open(filename.c_str(), "r");
    vector<string> rval; 
    while(count-- > 0) { 
        string l = getNextLine(f);
        if(l == "") break;
        rval.push_back(l);
    }
    return rval;
}
void FileLineLogger::trimLinesFromFrontCopy(int count) { 
    lineCount = getLines();
    if (count == 0) {
        lineCount = 0;
        return;
    }

    vector<string> toRemove = getFirstLines(count);
    if (toRemove.size() == 0) {
        lineCount = 0;
        LittleFS.remove(filename.c_str());
        return;
    }
    int bytesToRemove = 0;
    for(auto l : toRemove) {
        bytesToRemove += l.length() + 1;
        lineCount--;
    }
    string tempfile = filename + ".T";
    fs::File f1 = LittleFS.open(filename.c_str(), "r");
    fs::File f2 = LittleFS.open(tempfile.c_str(), "w");
    f1.seek(bytesToRemove);
    while(true) {
        uint8_t buf[64];
        int n = f1.read(buf, sizeof(buf));
        if (n <= 0) break;
        f2.write(buf, n);
    }
    f1.close();
    f2.close();
    LittleFS.remove(filename.c_str());
    LittleFS.rename(tempfile.c_str(), filename.c_str());
}

// works in simulation, screws up on hardware 
void FileLineLogger::trimLinesFromFront_ZeroPad(int count) { 
    lineCount = getLines();
    if (count == 0) {
        LittleFS.remove(filename.c_str());
        return;
    }
    fs::File f = LittleFS.open(filename.c_str(), "r+");
    int origSize = getTotalBytes();
    vector<string> toRemove = getFirstLines(count);
    int bytesToRemove = 0;
    for(auto l : toRemove) {
        bytesToRemove += l.length() + 1;
        lineCount--;
    }
    OUT("");
    if (bytesToRemove == origSize) { 
        lineCount = 0;
        f.close();
        LittleFS.remove(filename.c_str());
        return;
    }
    OUT("");
    int pos = 0;
    int fileSz = f.size();

    fs::File f2 = LittleFS.open(filename.c_str(), "r+");
    while(true) { 
        wdtReset();
        //OUT("%d %d %d %d", pos, bytesToRemove, origSize, fileSz);
        uint8_t buf[256];
        f.seek(pos + bytesToRemove);
        int n = f.read(buf, sizeof(buf));
        OUT("read() returned %d", n);
        if (n <= 0) break;
        f2.seek(pos);
        f2.write(buf, n);
        pos += n;
    }
    f2.close();
    OUT("");

    // can't truncate, just pad the end with zeros
    // f.truncate(origSize - bytesToRemove);
    {    
        f.seek(origSize - bytesToRemove);  
        pos = origSize - bytesToRemove;; 
        int fileSz = f.size();
        OUT("file size %d", fileSz);
        while(pos < fileSz) { 
            uint8_t c = '\0';
            f.write(&c, 1);
            pos++;
        }
    }
    OUT("");
}

void FileLineLogger::trimLinesFromFront(int count) { trimLinesFromFrontCopy(count); }
//void trimLinesFromFront(int count) { trimLinesFromFront_ZeroPad(count); }


int FileLineLogger::size() { return getLines(); } 
int FileLineLogger::getTotalBytes() {
    fs::File f = LittleFS.open(filename.c_str(), "r");
    int count = 0;
    while(true) { 
        char c;
        int n = f.read((uint8_t *)&c, 1);
        if (n != 1) break;
        if (c != '\0') count++;
    }
    return count;
}
int FileLineLogger::getLines() {
    if (lineCount < 0) { 
        fs::File f = LittleFS.open(filename.c_str(), "r");
        lineCount = 0;
        while(true) { 
            char c;
            int n = f.read((uint8_t *)&c, 1);
            if (n != 1) break;
            if (c == '\n') lineCount++;
        }
    }
    return lineCount;
}


void BatchWebLogger::checkInit() { 
    if (initialized) return;
    initialized = true;
    if (getResetReason(0) != 5 && spiffsReportLog.size() > 0) { 
        // stale logs of unknown time.  Find largest TSLP value and artificially set
        // firstLogAgeMs to that far in the past.
        string line;
        int lineNr = 0;
        uint32_t maxTslp = 0;
        OUT("Found %d stale entries in log, reading...", spiffsReportLog.size());
        while((line = spiffsReportLog.getLine(lineNr++)) != "") {

            wdtReset();
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, line);
            maxTslp = max(maxTslp, doc[TSLP].as<uint32_t>());
        }
        OUT("Found %d stale entries in log, setting TSLP to %d", spiffsReportLog.size(), maxTslp);
        firstLogAgeMs.set(maxTslp);
    }
}
BatchWebLogger::BatchWebLogger(const string &prefix/* = ""*/) : 
    spiffsReportLog(string("/") + prefix + ".reportLog"), 
    postFailTimer("BatchWebLogger", string("/") + prefix + ".pstFailTmr") {
}

JsonDocument BatchWebLogger::post(JsonDocument hdrDoc) {
    JsonDocument rval; 
    if (!wifiConnect()) {
        // broken you can't reset the report timer with TSLP values still in the log
        // TODO: add seperate postPeriodMin for timing reports and firstLogTs 
        // for measuring the age of reports.  firstLogTs is only reset when the log is empty
        // and the first post is made   
        // If stale logs are found after a hard boot, firstLogTs can be artificially set to be in
        // the past by the amount of the largest TSLP value found in the stale long
        // 
        // maybe firstLogAgeMin would be clearer
        OUT("SleepyLogger connection failed");
        postFailTimer.reportStatus(false);
        return rval;
    }
    
    String ssid = WiFi.SSID();
    String ip = WiFi.localIP().toString();
    String mac = getMacAddress();
    hdrDoc["GIT"] = GIT_VERSION;
    hdrDoc["MAC"] = mac.c_str(); 
    hdrDoc["SSID"] = ssid.c_str();
    hdrDoc["IP"] =  ip.c_str();
    hdrDoc["RSSI"] = WiFi.RSSI();
    //adminDoc["AVER"] = ESP_ARDUINO_VERSION_STR;

    HTTPClient client;
    client.setTimeout(10000); // default is 5000
    client.setConnectTimeout(10000); // default is 5000
    const string url = getServerName() + "/log";
    OUT("Connecting to %s...", url.c_str());
    int r = client.begin(url.c_str());
    client.addHeader("Content-Type", "application/json");
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);


    bool fail = false;
    const int batchSize = 10;
    while(spiffsReportLog.size() > 0) {
        fflush(stdout);
        uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
        string post;
        {   // add scope to free up admin string when we're done with it
            string admin;
            serializeJson(hdrDoc, admin);
            post = "{\"ADMIN\":" + admin + ",\"LOG\":[";
        }
        int i = 0;
        bool firstLine = true;
        vector<string> lines = spiffsReportLog.getFirstLines(batchSize);
        for(auto i : lines) { 
            fflush(stdout);
            uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, i);
            if (!error && doc[TSLP].as<int>() != 0) {
                uint32_t fpa = firstLogAgeMs.elapsed();
                doc["LTO"] = round((fpa - doc[TSLP].as<int>()) / 1000.0, .1);
                string s;
                serializeJson(doc, s);
                if (!firstLine) post += ",";
                firstLine = false;
                post += s;
            } 
        }
        post += "]}";
        floatRemoveTrailingZeros(post);

        // reserialize the digest and remove TSLP values 
        JsonDocument tmp;
        DeserializationError error = deserializeJson(tmp, post);
        JsonArray a = tmp["LOG"].as<JsonArray>();
        // leave TLSP in for debugging use for now
        //for(JsonVariant v : a) v.remove(TSLP);
        serializeJson(tmp, post);
        //OUT("XXXX deepsleepMs: %09.1f", (int)logTimestampMs.elapsed()/1000.0);

        for(int retry = 0; retry < 5; retry ++) {
            wdtReset();
            OUT("Posting...");
            r = client.POST(post.c_str());
            String resp =  client.getString();
            deserializeJson(rval, resp.c_str());

            // Print the log line to serial for data plotting tools 
            uint64_t nowmsec = (uint64_t)logTimestampMs.millis() + 52ULL * 365ULL * 24ULL * 3600ULL * 1000ULL;
            time_t nt = nowmsec / 1000ULL;
            struct tm *ntm = localtime(&nt);
            char buf[64];
            //2025-03-27T03:53:24.568Z
            if (r != 200) Serial.printf("ERROR:"); // prefix to spoil the line for data plotting tools 
            strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", ntm);
            Serial.printf("\n[%s.%03dZ] ", buf, (int)((nowmsec % 1000)));
            Serial.println(post.c_str());
            OUT("http.POST returned %d: %s", r, resp.c_str());
            if (r == 200) 
                break;
            if (retry >= 1) { 
                client.end();
                //wifiDisconnect();
                //wifiConnect();
                client.begin(url.c_str());
                client.addHeader("Content-Type", "application/json");
            }
            delay(1000);
        }
        if (r != 200) {  
            fail = true;
            break;
        }

        // runs out of memory here
        //vector<string> logs = spiffsReportLog;
        //logs.erase(logs.begin(), logs.begin() + i);
        //spiffsReportLog = logs;
        spiffsReportLog.trimLinesFromFront(batchSize);
    }
    client.end();
    postPeriodTimer.reset();
    postFailTimer.reportStatus(fail == false);

    //if (fail == true) { 
    //    OUT("SleepyLogger repeat posts failed");
    //    return rval;
    //}

    const char *ota_ver = rval["ota_ver"];
    if (ota_ver != NULL) { 
        if (strcmp(ota_ver, GIT_VERSION) == 0 || strlen(ota_ver) == 0
            // dont update an existing -dirty unless ota_ver is also -dirty  
            //|| (strstr(GIT_VERSION, "-dirty") != NULL && strstr(ota_ver, "-dirty") == NULL)
            ) {
            OUT("OTA version '%s', local version '%s', no upgrade needed", ota_ver, GIT_VERSION);
        } else { 
            OUT("OTA version '%s', local version '%s', upgrading...", ota_ver, GIT_VERSION);
            string url = getServerName() + "/ota";
            webUpgrade(url.c_str());
        }       
    }

    wifiDisconnect();
    return rval;
}

JsonDocument BatchWebLogger::log(JsonDocument doc, JsonDocument adminDoc, 
    bool forcePost /*= false*/, bool inhibitPost /*= false*/) {
    //OUT("%09.3f log() forcePost %d, reportTimer %.1f postPeriodMinutes %.1f", deepsleepMs.millis()/1000.0, 
    //forcePost, postPeriodTimer.elapsed()/1000.0, postFailTimer.getWaitMinutes());

    checkInit();
    if (spiffsReportLog.size() == 0)
        firstLogAgeMs.reset();
    JsonDocument result; 
    doc[TSLP] = firstLogAgeMs.elapsed();
    string s;
    serializeJson(doc, s);    
    //vector<string> logs = spiffsReportLog;
    //logs.push_back(s);
    //while(logs.size() > maxLogSize)
    //    logs.erase(logs.begin());
    //spiffsReportLog = logs;
    if (spiffsReportLog.size() > maxLogSize - 1)
        spiffsReportLog.trimLinesFromFront(spiffsReportLog.size() - (maxLogSize - 1));
    spiffsReportLog.push_back(s);
    
    float waitMs = postFailTimer.getWaitMinutes() * 60 * 1000;
    bool doPost = forcePost && !inhibitPost;
    if (postPeriodTimer.elapsed() > waitMs)
        doPost = true;
    if (spiffsReportLog.size() == maxLogSize)
        doPost = true; 
    if (LittleFS.usedBytes() > LittleFS.totalBytes() / 2 - 20 *1024)
        doPost = true; // post if space is running low 
    if (postFailTimer.failCount() > 0 && postPeriodTimer.elapsed() < waitMs) 
        doPost = false; // if theres been a failure, we don't post for any reason other than timer
    if (doPost) 
        result = post(adminDoc);
    
    return result;
}
    
float FailRetryInterval::getWaitMinutes(float defaultMin/* = -1*/) {
    if (defaultMin >= 0) 
        defaultWaitMin = defaultMin;
    float waitMin = defaultWaitMin;

    for(auto i : failStrategy) {
        FailActions::FailAction action = i.second;
        if (spiffsConsecutiveFails < i.first)
            continue;
        if (spiffsConsecutiveFails == i.first) { 
            if (action.reboot) ESP.restart();
            if (action.halt) { /*TODO*/}
            action.func();
        }
        // TODO BROKEN: these actions incorrectly apply to ALL later fail counts,
        // should only apply to the failures between *i and the next rule
        if (spiffsConsecutiveFails >= i.first ) { 
            if (action.waitMin != -1) waitMin = action.waitMin;
            if (action.increase != -1) {

                waitMin *= action.increase * (spiffsConsecutiveFails - i.first + 1);
            }
            if (action.multiply != -1) {
                for(int n = 0; n < spiffsConsecutiveFails.read() - i.first + 1; n++) 
                    waitMin *= action.multiply;
            }
        }
    } 
    return waitMin; 
} 

void FailRetryInterval::reportStatus(bool success) { 
    spiffsConsecutiveFails = success ? 0 : spiffsConsecutiveFails + 1;
    if (spiffsConsecutiveFails > 1) { 
        OUT("FailRetryInterval: %s repeated failures (%d)", name.c_str(), (int)spiffsConsecutiveFails);
    }
}

FailRetryInterval::FailRetryInterval(const string &_name/* = ""*/, const string &prefix /*= ""*/, float _defaultWaitMin/* = 1*/) 
     : defaultWaitMin(_defaultWaitMin), name(_name),
     spiffsConsecutiveFails(string("/") + prefix + "FRI.fails", 0) {
}
#endif // ESP32
