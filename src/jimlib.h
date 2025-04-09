#ifndef INC_JIMLIB_H
#define INC_JIMLIB_H
#include <functional>
#include <string>
#include <vector>
#include <stdarg.h>
#include <cstdint>
#include <regex>
#ifndef CSIM
#include "ArduinoOTA.h"
#include "WiFiUdp.h"
#include "driver/adc.h"
#include <rom/uart.h>

// starting jimlib.cpp cleanup
// Sketch uses 1346572 bytes (68%) of program storage space. Maximum is 1966080 bytes.
// real	0m16.728s

#ifdef ESP32
#include <WiFi.h>	
//#include <SPIFFS.h>
#else // ESP32
static inline void ledcSetup(int, int, int) {}
static inline void ledcAttachPin(int, int) {}
static inline void ledcWrite(int, int) {}
#endif //ESP32
#else // !CSIM
#include "ESP32sim_ubuntu.h"
#endif

#define DEG2RAD(x) ((x)*M_PI/180)
#define RAD2DEG(x) ((x)*180/M_PI)

using std::vector;
using std::string;
using std::pair;

void wdtInit(int sec);
void wdtReset();
void ledcInit(int pin, int freq, int res, int channel); 

String Sfmt(const char *format, ...);
std::string vsfmt(const char *format, va_list args);
std::string sfmt(const char *format, ...);
//template<typename... Args> std::string strfmt(const char *format, Args... args) { return sfmt(format, args...); }
#define strfmt sfmt 

int scanI2c();
void printPins();

class LineBuffer {
public:
	char line[1024];
	char len = 0;
	int add(char c, std::function<void(const char *)> f = NULL);
	void add(const char *b, int n, std::function<void(const char *)> f);
	void add(const uint8_t *b, int n, std::function<void(const char *)> f); 
};

class IntervalTimer {
public: 
	double last, interval;
	IntervalTimer(double i) : interval(i) {}
	bool tick(double now) {
		bool rval = (floor(now / interval) != floor(last / interval));
		last = now;
		return rval;
	}
};

class EggTimer {
	uint64_t last;
	uint64_t interval; 
	bool first = true;
	bool immediate = false;
public:
	EggTimer(float ms, bool imm = false) : interval(ms * 1000), last(0) { 
		reset(imm); 
	}
	bool tick() { 
		uint64_t now = micros();
		if (now - last >= interval || immediate == true) {
			immediate = false; 
			last += interval;
			// reset last to now if more than one full interval behind 
			if (now - last >= interval) 
				last = now;
			return true;
		} 
		return false;
	}
	void reset(bool imm = false) { 
		last = micros();
		immediate = imm;
	}
	void alarmNow() { 
		reset(true);
	}
};
typedef EggTimer Timer;

#ifdef ESP32
std::vector<std::string> split(const std::string &s, char delim);
#endif //#ifdef ESP32


struct DsTempData { 
	uint64_t id;
	uint64_t time;
	float degC;
};

class OneWireNg; 
std::vector<DsTempData> readTemps(OneWireNg *ow);

static inline const String &basename_strip_ext(const char *fn) {
	static String rval;
	rval = fn; 
	char *p = strrchr((char *)fn, '/');
	if (p != NULL) 
		rval = p + 1;
	if (rval.indexOf('.')) {
		rval = rval.substring(0, rval.indexOf('.'));
	}
	return rval;
}

static inline string &getMacAddress(string &result) {
	static uint8_t baseMac[6] = {0xff};
#ifdef ESP32
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
#endif
	static char baseMacChr[32] = {0};
	static String mac;
	snprintf(baseMacChr, sizeof(baseMacChr), "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	result = baseMacChr;
	return result;
}

const String &getMacAddress();

template<class T> bool fromString(const string &s, T&v);
template<> inline bool fromString(const string &s, int &v) { return sscanf(s.c_str(), "%d ", &v) == 1; }
template<> inline bool fromString(const string &s, uint64_t &v) { return sscanf(s.c_str(), "%lu ", &v) == 1; }
template<> inline bool fromString(const string &s, uint32_t &v) { return sscanf(s.c_str(), "%u ", &v) == 1; }
template<> inline bool fromString(const string &s, int64_t &v) { return sscanf(s.c_str(), "%ld ", &v) == 1; }
template<> inline bool fromString(const string &s, float &v) { return sscanf(s.c_str(), "%f ", &v) == 1; }
template<> inline bool fromString(const string &s, string &v) { v = s; return true; }
template<> inline bool fromString(const string &s, bool &v) { v = (s == "true"); return true; }

template<class T> string toString(const T&v);
template<> inline string toString(const uint64_t &v) { return sfmt("%lu ", v); }
template<> inline string toString(const uint32_t &v) { return sfmt("%u ", v); }
template<> inline string toString(const int64_t &v) { return sfmt("%ld ", v); }
template<> inline string toString(const int &v) { return sfmt("%d ", v); }
template<> inline string toString(const float &v) { return sfmt("%f ", v); }
template<> inline string toString(const string &v) { return v; }
template<> inline string toString(const bool &v) { return v ? "true" : "false"; }

#define LP() printf("%09.3f %s:%d\n", millis() / 1000.0, basename(__FILE__), __LINE__)
template<> inline bool fromString(const string &s, std::vector<string> &v) { 
	v = split(s, '\n');
	return true; 
}

template<> inline string toString(const std::vector<string> &v) { 
	string rval;
	int totalSize = 0;
	for(auto line : v) totalSize += line.length() + 1;
	rval.reserve(totalSize);
    for(auto line : v) rval += line + "\n";
    return rval;
}

//extern int SpiffsInit;
class SPIFFSVariableESP32Base { 
protected:
	SPIFFSVariableESP32Base();
	string filename, defaultStringValue;
	void writeAsString(const string &s);
	string readAsString();	
	static bool initialized;
protected:
	bool successfullyWritten = false;
public:
	bool debug = false;
	static void begin();
};

template<class T> 
class SPIFFSVariableESP32 : public SPIFFSVariableESP32Base { 
	const T def;
	T val;
	bool firstRead = true;
public:
	SPIFFSVariableESP32(const char *f, const T &d) : def(d), val(d) {
		filename = f;
		defaultStringValue = toString(def);
	}
	SPIFFSVariableESP32(const string &f, const T &d) : def(d), val(d) {
		filename = f;
		defaultStringValue = toString(def);
	}
	T read() {
		if (!initialized && debug) {
			printf("WARNING: early read from '%s'\n", filename.c_str());
		}
		if (firstRead) { 
			val = def;
			string s = readAsString();
			fromString(s, val);
			firstRead = false;
		}
		return val;
	}
	operator const T() { return read(); } 
	SPIFFSVariableESP32 & operator=(const T&v) { write(v); return *this; } 
	void write(const T &v) {
		if (!initialized) {
			printf("WARNING: early write to '%s'\n", filename.c_str());
		}
		val = v;
		string s = toString(val);
		this->writeAsString(s);
	}
};

// TODO: I think there is a ESP8266 alternative to the SPIFFS.h, find it 
template<class T> 
struct SPIFFSVariableFake { 
	const T def;
	SPIFFSVariableFake(const char *f, const T &d) : def(d) {}
	operator const T() { return def; } 
	SPIFFSVariableFake & operator=(const T&v) { return *this; }
};

#ifdef ESP32
#define SPIFFSVariable SPIFFSVariableESP32
#else
#define SPIFFSVariable SPIFFSVariableFake
#endif

class JimWiFi { 
	EggTimer report = EggTimer(1000);
	bool firstRun = true, firstConnect = true;
	std::function<void(void)> connectFunc = NULL;
	std::function<void(void)> otaFunc = NULL;
	// TODO: move this into JimWifi
	SPIFFSVariable<int> lastAP = SPIFFSVariable<int>("/lastap", -1);
public:
	struct ApInfo {
		const char *name;
		const char *pass;
		const char *mqtt;
	};
	std::vector<ApInfo> aps = {	{"Ping-582B", "", ""}, 
			{"ChloeNet", "niftyprairie7", ""},
			{"MOF-Guest", "", ""},
			{"ClemmyNet","clementine is a cat", "192.168.68.138"},
			{"ChloeNet4", "niftyprairie7", ""},
			{"Station 54", "Local1747", ""}, 
		};

	bool waitConnected(int ms, int bestMatch = -1) { 
		uint32_t startMs = millis();
		while(millis() - startMs < ms) { 
			if (WiFi.status() == WL_CONNECTED) {
				Serial.printf("Connected\n");
				if (bestMatch >= 0) lastAP = bestMatch;
				return true;
			}
			wdtReset();
			delay(100);
		}
		return false;
	}
	const char *getMqttServer() { 
		if (lastAP >= 0) return aps[lastAP].mqtt;
		return NULL;
	}
	void autoConnect() {
		if (!enabled) 
			return;
		//WiFi.disconnect(true);
		//WiFi.mode(WIFI_STA);
		//WiFi.setSleep(false);
		//delay(100);

		int bestMatch = lastAP;
		if (bestMatch >= 0 && bestMatch < aps.size()) { 
			Serial.printf("Trying cached WiFi AP '%s'...\n", aps[bestMatch].name);
			WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
			if (waitConnected(2000)) return;
			WiFi.disconnect();
			WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
			if (waitConnected(2000)) return;
		}
		bestMatch = -1;

		Serial.println("Scanning...");
		WiFi.disconnect(true);
		//WiFi.mode(WIFI_STA);
		//WiFi.setSleep(false);
		//delay(1000);
		wdtReset();
		int n = WiFi.scanNetworks();
		wdtReset();
		Serial.println("scan done");
		
		if (n == 0) {
			Serial.println("no networks found");
		} else {
			Serial.printf("%d networks found\n", n);
			for (int i = 0; i < n; ++i) {
			// Print SSID and RSSI for each network found
			// TODO handle the trailing space in some AP names 
				Serial.printf("%3d: %s (%d)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
				for (int j = 0; j < aps.size(); j++) { 
					if (strcmp(WiFi.SSID(i).c_str(), aps[j].name) == 0) { 
						if (bestMatch == -1 || j < bestMatch) {
							bestMatch = j;
						}
					}
				}	
			}
		}
		if (bestMatch == -1) {
			bestMatch = 0;
		}
		WiFi.scanDelete();
		Serial.printf("Using WiFi AP '%s'...\n", aps[bestMatch].name);
		WiFi.disconnect();
		WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
		if (waitConnected(2000, bestMatch)) return;
		WiFi.disconnect();
		WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
		waitConnected(12000, bestMatch); 
	}
public:
    bool updateInProgress = false;
	bool debug = false;
	bool enabled = true;
	WiFiUDP udp;
	//WiFiMulti wifi;
	EggTimer connCheck = EggTimer(30000);
	void onConnect(std::function<void(void)> oc) { 
		connectFunc = oc;
	}
	void onOTA(std::function<void(void)> oo) { 
		otaFunc = oo;
	}
	void invalidateCachedAP() { 
		lastAP = -1;
	}
	void run() {
		if (!enabled) 
			return;

		if (firstRun) {
			autoConnect();			
			firstRun = false;
		}
		if (firstConnect == false && connCheck.tick() && WiFi.status() != WL_CONNECTED) {
			WiFi.disconnect();
			WiFi.reconnect();
		}
		if (WiFi.status() == WL_CONNECTED) { 
			if (firstConnect ==  true) { 
				firstConnect = false;

				ArduinoOTA.onStart([this]() {
					String type;
					if (this->otaFunc != NULL) { 
						this->otaFunc();
					}
					if (ArduinoOTA.getCommand() == U_FLASH) {
					type = "sketch";
					} else { // U_FS
					type = "filesystem";
					}
					#ifdef ESP32
					//WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector   
					#endif

					// NOTE: if updating FS this would be the place to unmount FS using FS.end()
					Serial.println("Start updating " + type);
					});
					ArduinoOTA.onEnd([]() {
					Serial.println("\nEnd");
				});
				ArduinoOTA.onProgress([&](unsigned int progress, unsigned int total) {
						//Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
						wdtReset();
						updateInProgress = true;
						delay(30);
				});
				ArduinoOTA.onError([&](ota_error_t error) {
					Serial.printf("Error[%u]: ", error);
					if (error == OTA_AUTH_ERROR) {
					Serial.println("Auth Failed");
					} else if (error == OTA_BEGIN_ERROR) {
					Serial.println("Begin Failed");
					} else if (error == OTA_CONNECT_ERROR) {
					Serial.println("Connect Failed");
					} else if (error == OTA_RECEIVE_ERROR) {
					Serial.println("Receive Failed");
					} else if (error == OTA_END_ERROR) {
					Serial.println("End Failed");
					}
					updateInProgress = false;
				});
	
				
				ArduinoOTA.begin();

				udp.begin(9999);
				if (connectFunc != NULL) { 
					connectFunc();
				}
			}
		
				
			do {
				ArduinoOTA.handle();
			} while(updateInProgress == true);
//			server.handleClient();
			if (report.tick()) {
				udpDebug(""); 
			}
		}
	}
	bool connected() { return WiFi.status() == WL_CONNECTED;  }  
	void udpDebug(const char *s) { 
		if (!connected() || !debug)
			return;
		udp.beginPacket("255.255.255.255", 9000);
		char b[128];
		snprintf(b, sizeof(b), "%08d %s %s " __DATE__ "   " __TIME__ "   %s ", 
			(int)(millis() / 1000), WiFi.localIP().toString().c_str(), 
			basename_strip_ext(__BASE_FILE__).c_str(), getMacAddress().c_str());
		udp.write((const uint8_t *)b, strlen(b));
		udp.write((const uint8_t *)s, strlen(s));
		udp.write((const uint8_t *)"\n", 1);
		udp.endPacket();
	}
};

#ifdef CSIM
#include <fcntl.h>
#include <unistd.h>
#endif

#if 0 
class ShortBootDebugMode {
	SPIFFSVariable<int> shortBootCount = SPIFFSVariable<int>("/shortBootCount", 1);
	bool initialized = false;
	bool cleared = false;
	int sbCount; 
	int threshold;
  public:
	ShortBootDebugMode(int t) : threshold(t) {}
	void begin() {
		shortBootCount = shortBootCount + 1;
		sbCount = shortBootCount;
		
		Serial.printf("Short boot count %d\n", sbCount);
		initialized = true;
	}
	bool check() {
		if (initialized == false) { 
			begin();
		}
		if (millis() > 5000 && cleared == false) { 
			cleared = true;
			shortBootCount = 0 ;
		}
		return sbCount >= threshold;
	} 
};
#endif //0
#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
#ifndef GIT_VERSION
#define GIT_VERSION "unknown-git-version"
#endif

void bin2hex(const char *in, int len, char *out, int olen);
int hex2bin(const char *in, char *out, int inLength);
std::string nmeaChecksum(const std::string &s);

void webUpgrade(const char *u);
float avgAnalogRead(int p, int avg = 1024);

// create a String from a char buf without NULL termination 
String buf2str(const byte *buf, int len);

// connects to an mqtt server, publish stuff on <name>, subscribe to <name>/in
class PubSubClient;
class MQTTClient { 
	WiFiClient espClient;
	void callBack(char *topic, byte *p, unsigned int l);
	std::function<void(String,String)> userCallback = NULL;
	PubSubClient *client = NULL;
public:
    String topicPrefix, server;
	bool active;
	void setCallback(std::function<void(String,String)> cb) { userCallback = cb; }
	void begin(const char *s, const char *t, std::function<void(String,String)> cb = NULL, bool a = true);
	MQTTClient();
	void publish(const char *suffix, const char *m);
	void publish(const char *suffix, const String &m);
	void pub(const char *m);
	void reconnect();
	void dprintf(const char *format, ...);
	void run();
};
int getLedPin();

#if 0
class PwmChannel {
	int pin; 
	int channel;
	int pwm = -1;
public:		
	int gradual;
	PwmChannel(int p, int hz = 50, int c = 0, int g = 0) : pin(p), channel(c), gradual(g) {
		//ledcSetup(channel, hz, 16);
		//ledcAttachPin(pin, channel);
		ledcInit(pin, hz, 16, channel);
	}
	void setMs(int p) { set(p * 4715 / 1500); };
	void setPercent(int p) { set(p * 65535 / 100); } 
	void setRaw(int p) { 
		while(gradual && pwm > 0 && pwm != p && pwm < 65536 ) { 
			pwm += pwm < p ? 1 : -1;
			ledcWrite(channel, pwm);  
			delayMicroseconds(gradual);
		}
		ledcWrite(channel, p); 
		pwm = p; 
	} 
	void set(int p) {  
		pattern = 0;
		setRaw(p);
	}
	float get() { return (float)pwm / 65535; } 

	int patternPeriod = 0, pattern = 0, patternLen = 0, patternCount = 0;
	float patternBrightness = 1.0, patternSoftness = 0;
	int lastPatIdx = 0, patternFudgeMs = 0;
	void setPattern(int ms, int pat, float bright = 1.0, int count =-1) { 				
		patternPeriod = ms;
		pattern = pat;
		patternLen = 0;
		patternCount = count;
		patternBrightness = bright;
		lastPatIdx = 0;
		while(pat) { 
			patternLen++;
			pat = pat >> 1;
		}
		patternFudgeMs = millis() % (patternPeriod * patternLen);
	}
	void run() { // not reentrant, this can crash if isr routines call setPattern() during loop()->run()
		if (patternPeriod == 0 || pattern == 0)
			return;
		int pat = pattern;
		int pi = ((millis() - patternFudgeMs) % (patternPeriod * patternLen)) / patternPeriod;
		for (int n = 0; n < min(16, pi); n++) { 
			pat= pat >> 1;
		}
		if (pat & 0x1) { 
			float v = abs((int)(millis() % patternPeriod) - patternPeriod/2.0) / (patternPeriod / 2.0 / 100);
			v = min(100.0, max(0.0, 100.0 - v * patternBrightness));
			setRaw(v * 65535 / 100);
		} else { 
			setRaw(0);
		}
		if (pi == 0 && lastPatIdx != 0 && patternCount > 0 && --patternCount == 0) {
			pattern = patternPeriod = patternCount = 0;
			set(0);
		}
		lastPatIdx = pi;
	}
};
#endif 

#if 1 // TODO - this could move back to supporting ESP8266
using std::smatch;

// TODO: replace "get xxx" hook with making = and white space optional in 
// "set xxx" hook, just use "set xxx" to show command
// TODO: use case insensitive regex matches

class CommandLineInterfaceESP32 { 
	typedef std::function<string(const char *,std::smatch)> callback;
	std::vector<std::pair<std::string, callback>> handlers;
public:

	CommandLineInterfaceESP32() {}

	void on(const char *pat, callback h) { 
		handlers.push_back(std::pair<std::string, callback>(pat, h));
	}
	void on(const char *pat, std::function<void()> f) { 
		on(pat, [f](const char *, std::smatch) { f(); return string(); });
	}
	void on(const char *pat, std::function<void(std::smatch)> f) { 
		on(pat, [f](const char *, std::smatch m) { f(m); return string(); });
	}
	void on(const char *pat, std::function<void(const char *l)> f) { 
		on(pat, [f](const char *l, std::smatch) { f(l); return string(); });
	}
	string process(const char *line) {
		string rval = ""; 
		if (strstr(line, "hooks")) { 
			smatch dummy;
			for(auto i = handlers.begin() + 1; i != handlers.end(); i++) { 
				rval += i->first + " \t " + i->second("", dummy) + "\n";
			}
			return rval;
		}
		for(auto i = handlers.begin(); i != handlers.end(); i++) { 
			std::smatch res;
			std::regex exp((i->first).c_str(), std::regex::icase);
			std::string str = line;
			if (std::regex_match(str, res, exp))  
				rval += i->second(line, res);
		}
		return rval;
	}
	template<typename T>
	void hookRaw(const char *pat, T* v) {
		const char *fmt = formatOf<T>();
		on(pat, [v,fmt](const char *, smatch m) { 
			if (m.size() > 1) 
				sscanf(m.str(1).c_str(), fmt, v);
			return strfmt(fmt, *v);
		});
	}

	template<typename T>
	void hookVar(const char *l, T*p) {
		std::string s = strfmt("set %s[= ]*(.*)", l); // TODO make = optional 
		hookRaw(s.c_str(), p);
		//s = strfmt("get %s", l);
		//hookRaw(s.c_str(), p);	
	}

	template<typename T> const char *formatOf();
};

template<> inline const char *CommandLineInterfaceESP32::formatOf<float>() { return "%f"; }
template<> inline const char *CommandLineInterfaceESP32::formatOf<int>() { return "%i"; }
template<> inline const char *CommandLineInterfaceESP32::formatOf<unsigned int>() { return "%x"; }

template<>
inline void CommandLineInterfaceESP32::hookRaw<string>(const char *pat, string *v) {
	on(pat, [v](const char *, smatch m) { 
		if (m.size() > 1 && m.str(1).length() > 0)
			*v = m.str(1).c_str();
		return string(*v);
	});
}
template<>
inline void CommandLineInterfaceESP32::hookRaw<String>(const char *pat, String *v) {
	on(pat, [v](const char *, smatch m) { 
		if (m.size() > 1 && m.str(1).length() > 0)
			*v = m.str(1).c_str();
		return string((*v).c_str());
	});
}

// TODO: need to reimplement this without std::regex so that ESP8266 can still fit in OTA
struct CommandLineInterfaceESP8266 { 
	typedef std::function<string(const char *,std::smatch)> callback;
	void run();
	void begin();
	void on(const char *pat, callback h) {}
	void on(const char *pat, std::function<void()> f) {}
	void on(const char *pat, std::function<void(std::smatch)> f) {}
	void on(const char *pat, std::function<void(const char *l)> f) {}
	string process(const char *line) {
		string s = "TODO: CommandLineInterfaceESP8266 not implemented, ignoring: ";
		s += line;
		//mqtt.pub(s.c_str());
		Serial.print(s.c_str()); 
		return s; 
	}
	template<typename T>
	void hookVar(const char *l, T*p) {}
};

#ifdef ESP32
typedef CommandLineInterfaceESP32 CommandLineInterface;
#else
typedef CommandLineInterfaceESP8266 CommandLineInterface;
#endif

template<typename T> 
class CliVariable {
	T val;
  public:
	CliVariable(CommandLineInterface &c, const char *n) { c.hookVar(n, &val); }
	CliVariable(CommandLineInterface &c, const char *n, T v) : val(v) { c.hookVar(n, &val); }
	operator T&() { return val; }
	T& operator =(T v) { val = v; return val; } 
};

class QuickRebootCounter {
	SPIFFSVariable<int> count = SPIFFSVariable<int>("/quickRebootCount", 0); 
	SPIFFSVariable<int> pending = SPIFFSVariable<int>("/quickRebootPending", 0); 
public:
	QuickRebootCounter() { 
		if (pending == 1) count = count + 1;
		pending = 1;
	}
	void run() { 
		if (millis() > 1000) {
			count = 0; pending = 0; 
		}
	}
	int reboots() { return count; }
};

#define CLI_VARIABLE_INT(name,val) CliVariable<int> name = CliVariable<int>(j.cli, #name, val)
#define CLI_VARIABLE_FLOAT(name,val) CliVariable<float> name = CliVariable<float>(j.cli, #name, val)
#define CLI_VARIABLE_STRING(name,val) CliVariable<String> name = CliVariable<String>(j.cli, #name, val)
#define CLI_VARIABLE_HEXINT(name,val) CliVariable<unsigned int> name = CliVariable<unsigned int>(j.cli, #name, val)

class JStuff {		
public:
	bool parseSerial;
	bool beginRan = false;
	std::function<void()> onConn = NULL;
	LineBuffer lb;
	bool debug = false;
	CommandLineInterface cli;
	CliVariable<int> logLevel;
	//QuickRebootCounter quickRebootCounter;
public:
	//PwmChannel led = PwmChannel(getLedPin(), 1024, 10, 2);
	struct {
		void run() {}
		void setPattern(int, int) {}
		void setPercent(int) {}
	} led;
	JStuff(bool ps = true) : parseSerial(ps), logLevel(cli, "logLevel", 1) {
		cli.on("DEBUG", [this]() { 
			jw.debug = debug = true; 
		});
	}
	uint64_t thisRun, lastRun;
	bool forceTicksNow = false; 
	bool hz(float hz) { 
		int us = 1000000.0 / hz;
		return forceTicksNow || (thisRun / us != lastRun / us);
	}
	bool secTick(float sec) { 
		return 
		hz(1.0/sec);
	}
	void forceTicks() { forceTicksNow = true; }

	bool onceFlag = true;
	bool once() { 
		bool rval = onceFlag;
		onceFlag = false;
		return rval;
	}

	bool cliEcho = true;
	JimWiFi jw;
	MQTTClient mqtt;
	void run() { 
		if (beginRan == false)
			begin();
		//quickRebootCounter.run();
		lastRun = thisRun;
		thisRun = micros();
		forceTicksNow = false;

		wdtReset();
		jw.run(); 
		wdtReset();
		mqtt.run(); 
		led.run();
		while(parseSerial == true && Serial.available()) { 
			lb.add(Serial.read(), [this](const char *l) {
				string r = cli.process(l);
				if (cliEcho) 
					Serial.println(r.c_str());
			});
		}
	}
	void begin() { 
		beginRan = true;
		wdtInit(15);
		SPIFFSVariableESP32Base::begin();

		Serial.begin(115200);
		Serial.printf("BOOT %s git:" GIT_VERSION " mac:%s time:%05.3fs built:" __DATE__ " " __TIME__ " \n", 
			basename_strip_ext(__BASE_FILE__).c_str(), getMacAddress().c_str(), millis() / 1000.0);
		getLedPin();
		//led.setPercent(30);
		jw.onConnect([this](){
			led.setPattern(500, 2);
			const char *mqttServer = jw.getMqttServer();
			if (mqttServer != NULL && strlen(mqttServer)) {
				 mqtt.begin(mqttServer, basename_strip_ext(__BASE_FILE__).c_str());
			}
			if (WiFi.SSID() == "ClemmyNet" || WiFi.SSID() == "FakeWiFi") {
				jw.debug = mqtt.active = true;  
				out("JStuff: forcing debug and mqtt.active due to WiFi network");
 			}
			Serial.printf("Connected to AP '%s' in %dms, IP=%s, channel=%d, RSSI=%d\n",
				WiFi.SSID().c_str(), millis(), WiFi.localIP().toString().c_str(), WiFi.channel(), WiFi.RSSI());
			if (onConn != NULL) { 
				onConn();
			}
		});
		mqtt.setCallback([this](String t, String m) {
			string r = cli.process(m.c_str());
			if (cliEcho) 
				mqtt.pub(r.c_str());
		});
	}
	void out(const string &s) { 
		mqtt.pub(s.c_str());
		printf("%s", s.c_str());
		printf("\n");
		jw.udpDebug(s.c_str());
	}
	void out(const char *format, ...) { 
		va_list args;
		va_start(args, format);
		string s = vsfmt(format, args);
		va_end(args);
		out(s);
	}
	void log(int ll, const char *format, ...) { 
		if (logLevel < ll) 
			return; 
		va_list args;
		va_start(args, format);
		string s = vsfmt(format, args);
		va_end(args);
		mqtt.pub(s.c_str());
		Serial.println(s.c_str());
		//jw.udpDebug(buf);
	}
	void log(int l, String s) { log(l, s.c_str()); }
	void log(int l, std::string s) { log(l, s.c_str()); }
	void out(String s) { string s2 = s.c_str(); out(s2); }
};

//#define OUT j.out
//#define LOG j.log

#endif

#if 0 // should work, just move to jimlib.cpp
class TempSensor { 
	public:
	#define OWNG OneWireNg_CurrentPlatform 
	OWNG *ow = NULL; 
	TempSensor(OWNG *p) : ow(p) {}
	TempSensor(int pin, bool pullup = true) {
		ow = new OWNG(pin, pullup);
	}
	float readTemp(int retries = 5) { 
		for(int i = 0; i < retries; i++) { 
			std::vector<DsTempData> t = readTemps(ow);
			if (t.size() > 0) {
				return t[0].degC;
			}
		}	
		return -999.0;
	}
};
#endif 

static inline void digitalToggle(int pin) { pinMode(pin, OUTPUT); digitalWrite(pin, !digitalRead(pin)); }

#define PRINTLINE() if(1) { printf("%09.3f " __FILE__ " line %d\n", millis() / 1000.0, __LINE__); } 

const char *reset_reason_string(int reason);

class HzTimer { 
public:
	HzTimer(float h, bool forceFirst = true) : force(forceFirst), hertz(h) {}
	bool force;
	float hertz;
	uint32_t last = 0;
	bool hz(float h) {
		bool rval = force || (millis() - last) > 1000.0 / h;
			if (rval) {
				last = millis();
				force = false;
			}
		return rval;
	}
	bool tick() { return hz(hertz); }
	bool secTick(float sec) { return sec > 0 ? hz(1/sec) : true; }
};
	
int getResetReason(int cpu = 0);

class DeepSleepManager { 
	typedef std::function<void(uint32_t)> Callback;
	vector<Callback> callbacks; 	
public:
	void onDeepSleep(Callback f) { 
		callbacks.push_back(f); 
	}
	void prepareSleep(uint32_t ms) {
		for(auto f : callbacks) f(ms);
	}
	void deepSleep(uint32_t ms);
};

void wifiDisconnect();
bool wifiConnect();

extern DeepSleepManager &dsm();
static inline void deepSleep(uint32_t ms) { dsm().deepSleep(ms); }

class DeepSleepElapsedTimer { 
	static const int typicalBootTimeMs = 500;
    SPIFFSVariable<uint32_t> bootOffsetMs, startTs; 
    bool initialized = false, startExpired;
	string prefix;
	void checkInit();
public:
    DeepSleepElapsedTimer(const string &_prefix, bool _startExpired = false);
	void prepareSleep(uint32_t ms);
	uint32_t millis();
	void set(uint32_t ms);
	uint32_t elapsed() { return this->millis(); }
    void reset() { set(0); }
};

static inline float round(float f, float prec) {
    return floor(f / prec + .5) * prec;
}

const char *reset_reason_string(int reason);

//#endif
#endif //#ifndef INC_JIMLIB_H
