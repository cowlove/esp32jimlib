#pragma once 
#include "jimlib.h"
#include <ArduinoJson.h>
#ifndef CSIM
#include <FS.h>
#endif

using fs::File;

namespace FailActions {
    typedef std::function<void()> FailCallback;
    class FailAction { public: FailCallback func = [](){}; float waitMin = -1, increase = -1, multiply = -1; bool reboot = false, halt = false; };
    struct WaitMin : public FailAction { WaitMin(float minutes) { waitMin = minutes; }};
    struct IncreaseWait : public FailAction { IncreaseWait(float in) { increase = in; }};
    struct MultiplyWait : public FailAction { MultiplyWait(float mu) { multiply = mu; }};
    struct HardReboot : public FailAction { HardReboot() { reboot = true; }};
    struct Halt : public FailAction { Halt() { halt = true; }};
    struct Callback : public FailAction { Callback(FailCallback f) { func = f; }};
};

class FailRetryInterval {
    string prefix;
    SPIFFSVariable<int> spiffsConsecutiveFails;
    typedef vector<pair<int, FailActions::FailAction>> FailActionList;
    FailActionList failStrategy;
    string name;
public:
    int failCount() { return spiffsConsecutiveFails; }
    void reset() { spiffsConsecutiveFails = 0; }
    float defaultWaitMin;
    void setFailStrategy(FailActionList l) { failStrategy = l; };
    FailRetryInterval(const string &name = "", const string &prefix = "", float _defaultWaitMin = 1);
    void reportStatus(bool success);
    float getWaitMinutes(float defaultMin = -1);
};

class FileLineLogger { 
    string filename;
    int lineCount = -1;
    string getNextLine(fs::File &f);
public:
    FileLineLogger(const string &fn);
    void push_back(const string &s);
    string getLine(int lineNumber);
    vector<string> getFirstLines(int count);
    void trimLinesFromFrontCopy(int count);
    void trimLinesFromFront_ZeroPad(int count);
    void trimLinesFromFront(int count);
    int size();
    int getTotalBytes();
    int getLines();
};

class BatchWebLogger { 
    bool initialized = false;
    void checkInit();
    const char *TSLP = "TSLP"; // "Time Since Last Post" key/value to crease LTO "Log Time Offset" value in posted data 
public:
    FileLineLogger spiffsReportLog;
    std::function<string(void)> getServerName = [](){ return "http://hostname/post"; };
    FailRetryInterval postFailTimer;
    // currently runs out of memory at about 100 at line 'vector<string> logs = spiffsReportLog' in post()
    // only succeeds after reboot when set to 90 
    static const int maxLogSize = 150; 
    DeepSleepElapsedTimer postPeriodTimer = DeepSleepElapsedTimer("/bwl_postTimer", true);
    DeepSleepElapsedTimer firstLogAgeMs = DeepSleepElapsedTimer("/bwl_log0Age");
    DeepSleepElapsedTimer logTimestampMs = DeepSleepElapsedTimer("/bwl_ltst");

    BatchWebLogger(const string &prefix = "");
    JsonDocument post(JsonDocument adminDoc);    
    JsonDocument log(JsonDocument doc, JsonDocument adminDoc, bool forcePost = false, bool inhibitPost = false);
};
