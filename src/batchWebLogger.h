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

class GenericData { 
public:
    template <class T> 
    struct DataType {
        DataType() {}
        DataType(const T &v) : valid(true), value(v) {} 
        T value;
        bool valid = false;
        DataType &operator =(const T&v) { valid = true; value =v; return *this; }
    };
    DataType<float> _double;
    DataType<string> _string;
    DataType<int> _int;

    GenericData &operator =(double v) { _double = v; return *this;  } 
    GenericData &operator =(const string &v) { _string = v; return *this; } 
    GenericData &operator =(int v) { _int = v; return *this; } 
    template<class T> bool is();
    template<class T> T as();

    //template<> bool is<int>() { return _int.valid; }
    //template<> bool is<float>() { return _string.valid; }
    //template<> float as<float>() { return _float.value; }
    //template<> int as<int>() { return _int.value; }
    //template<> string as<string>() { return _string.value; }
};
template<> bool GenericData::is<double>() { return _double.valid; }
template<> bool GenericData::is<int>() { return _int.valid; }
template<> bool GenericData::is<string>() { return _string.valid; }
template<> int    GenericData::as<int>   () { return _int.value; }
template<> double GenericData::as<double> () { return _double.value; }
template<> string GenericData::as<string>() { return _string.value; }
    
class BatchWebLogger { 
    bool initialized = false;
    void checkInit();
    const char *TSLP = "TSLP"; // "Time Since Last Post" key/value to crease LTO "Log Time Offset" value in posted data 
    DataMap post(DataMap &adminDoc);    
public:
    typedef std::map<string, GenericData> DataMap;
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
    DataMap log(DataMap &doc, DataMap &adminDoc, bool forcePost = false, bool inhibitPost = false);
};
