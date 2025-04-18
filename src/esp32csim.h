#ifndef _Csim_UBUNTU_H_
#define _Csim_UBUNTU_H_
/* Simple library and simulation environment to compile and run an Arduino sketch as a 
 * standard C command line program. 
 * 
 * Most functionality is unimplemented, and just stubbed out.  Minimal simluated 
 * Serial/UDP/Interrupts/buttons are sketched in. 
 * 
 * Currently replaces the following block of Arduino include files:
 * 
 *
 */

#include <cstdint>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstring>
#include <string>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
#include <functional>
#include <regex>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <malloc.h>

using std::map;
using std::ios_base;
using std::vector;
using std::pair;
using std::ifstream;
using std::string;
using std::function;
using std::iterator;
using std::min;
using std::max;
using std::deque;
using std::to_string;

#define PROGMEM 
#define ARDUINO_VARIANT "csim"
#define ESP_ARDUINO_VERSION_STR "1.1.1"
#ifndef GIT_VERSION
#define GIT_VERSION "no-git-version"
#endif
#define byte char

extern uint64_t _micros; 
extern uint64_t _microsMax;
uint32_t micros();
uint32_t millis();
void delayMicroseconds(int);
inline static void delayUs(int x) { delayMicroseconds(x); } 

class Csim_Module {
public:
	Csim_Module();
	virtual void parseArg( char **&, char **) {}
	virtual void setup() {};
	virtual void loop() {};
	virtual void done() {};
};

class Csim_flags : public Csim_Module { 
	void addflag(const char *f) {
		if (strcmp(f, "OneProg") == 0) OneProg = true;
	}
public:
	bool OneProg = false;
	virtual void parseArg(char **&a, char **) override {
		if (strcmp(*a, "--csimFlag") == 0) addflag(*(++a));
	}
};

extern Csim_flags csim_flags;

class Csim {
public:
	int argc; 
	char **argv;
	uint64_t bootTimeUsec = 0;
	double seconds = -1;
	int resetReason = 0;
	bool showArgs;
	vector<Csim_Module *> modules;
	struct TimerInfo { 
		uint64_t last;
		uint64_t period;
		function<void(void)> func;
	};
	typedef vector<TimerInfo> timers;
	void delayMicroseconds(long long us);
	void main(int argc, char **argv);
	void parseArgs(int argc, char **argv);
	void exit();
};

void Csim_exit(); 

// Stub out FreeRTOS stuff 
typedef int SemaphoreHandle_t;
int xSemaphoreCreateCounting(int max, int init);
int xSemaphoreCreateMutex() ;
int xSemaphoreGive(int h) ;
int xSemaphoreTake(int h, int delay) ;
int uxSemaphoreGetCount(int h);

#define portMAX_DELAY 0 
#define tskIDLE_PRIORITY 0
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 0 	
inline static void xTaskCreate(void (*)(void *), const char *, int, void *, int, void *) {}
#define WRITE_PERI_REG(a, b) if(0) {}
#define RTC_CNTL_BROWN_OUT_REG 0

typedef int esp_err_t;
typedef int RESET_REASON; 
typedef struct { int timeout_ms, idle_core_mask, trigger_panic; } esp_task_wdt_config_t;  
inline static void esp_task_wdt_init(const esp_task_wdt_config_t *) {}
inline static void esp_task_wdt_init(int, int) {}
inline static void esp_task_wdt_deinit() {}
inline static void esp_task_wdt_reset() {}
inline static esp_err_t esp_task_wdt_add(void *) { return 0; }
inline static esp_err_t esp_task_wdt_delete(const void *) { return 0; }
int rtc_get_reset_reason(int);

#define ADC1_CHANNEL_1 1
#define ADC1_CHANNEL_2 2
#define ADC1_CHANNEL_3 3
inline static int adc1_get_raw(int) { return 0; }

// stub out Adafruit_NeoPixel lib 
#define NEO_GRB 0
#define NEO_KHZ800 0
struct Adafruit_NeoPixel {
  Adafruit_NeoPixel(int, int, int) {}
  static int Color(int, int, int) { return 0; }
  void begin() {}
  void setPixelColor(int, int a = 0, int b = 0, int c = 0) {}
  void show() {}
  void clear() {}
};

// stub out FS.h
namespace fs { 
class File {
	int fd = -1;
	string filename;
public: 
	File() {}
	File(const char *fn, const char *m) {
		mkdir("./spiff", 0755);
		filename = string("./spiff/") + fn;
		int mode = O_RDONLY;
		if (strcmp(m, "a") == 0) mode = O_CREAT | O_APPEND | O_WRONLY;
		if (strcmp(m, "r") == 0) mode = O_CREAT | O_RDONLY;
		if (strcmp(m, "w") == 0) mode = O_CREAT | O_TRUNC | O_WRONLY;
		if (strcmp(m, "r+") == 0) mode = O_CREAT | O_RDWR;
		if (strcmp(m, "w+") == 0) mode = O_CREAT | O_TRUNC | O_RDWR;
		fd = open(filename.c_str(), mode, 0644);
	}
	bool operator!() { return false; } 
	operator bool() { return true; } 
	File openNextFile(void) { return *this; }
	void close() { if (fd != -1) ::close(fd); fd = -1; }
    int print(const char *s) { return ::write(fd, s, strlen(s)); }
	int printf(const char *, ...) { return 0; } 
	int write(const char *d, int l) { return ::write(fd, d, l); } 
	int write(const uint8_t *d, int l) { return ::write(fd, d, l); } 
	int write(uint8_t c) { return ::write(fd, &c, 1); }
	int seek(int pos) { return ::lseek(fd, pos, SEEK_SET); } 
	int size() { 
		int fd2 = ::open(filename.c_str(), O_RDONLY);
		int size = lseek(fd2, 0, SEEK_END); 
		::close(fd2);
		return size;
	}
	int flush() { return 0; }	
	int read(uint8_t *buf, int len) { return ::read(fd, buf, len); }
	~File() { close(); } 
};
};

using fs::File;
struct FakeSPIFFS {
	const string root = string("./spiff");
	void begin() {}
	void format() {}
	File open(const char *f, const char *m) { return File(f, m); } 
	void rename(const char *oldname, const char *newname) { 
		string fn1 = root + oldname;
		string fn2 = root + newname;
		int n = ::rename(fn1.c_str(), fn2.c_str()); 
	}	
	void remove(const char *fn) { 
		string fn1 = root + fn;
		int n = ::remove(fn1.c_str()); 
	}
	int usedBytes() { return 20 * 1024; }
	int totalBytes() { return 115 * 1024; }
	int truncate(const char *f, int pos) { return ::truncate(f, pos); }
};

extern FakeSPIFFS SPIFFS, LittleFS;

// stub out ArduinoOTA.h
struct FakeArduinoOTA {
	void begin() {}
	void handle() {}
	int getCommand() { return 0; } 
	void onEnd(function<void(void)>) {}
	void onStart(function<void(void)>) {}
	void onError(function<void(int)>) {}
	void onProgress(function<void(int, int)>) {}
};

extern FakeArduinoOTA ArduinoOTA;

typedef int ota_error_t; 
#define OTA_AUTH_ERROR 0 
#define OTA_BEGIN_ERROR 0 
#define OTA_CONNECT_ERROR 0 
#define OTA_RECEIVE_ERROR 0 
#define OTA_END_ERROR 0 

// stub out Esp.h
struct FakeESP {
	uint32_t getFreeHeap() { 
		//struct mallinfo2 mi = mallinfo2();
		return 16 * 1024 * 1024;// - mi.uordblks;
	}
	uint32_t minFreeHeap = 0xffffffff;
	uint32_t getMinFreeHeap() { return minFreeHeap = min(minFreeHeap, getFreeHeap()); } 
	void restart() { Csim_exit(); }
	uint32_t getChipId() { return 0xdeadbeef; }
};

extern FakeESP ESP;

// stub out OneWireNg lib
struct OneWireNg {
	OneWireNg(int, int) {}
	typedef int ErrorCode; 
	typedef int Id[8];
	static const int EC_NO_DEVS = 0, EC_MORE = 1, EC_DONE = 0;
	void writeByte(int) {}
	void addressSingle(Id) {}
	void touchBytes(const unsigned char *, int) {}
	void searchReset() {}
	static int crc8(const unsigned char *, int) { return 0; }
	int search(Id) { return 0; }
};
typedef OneWireNg OneWireNg_CurrentPlatform;

void ledcWrite(int chan, int val);

struct ledc_channel_config_t {
	int gpio_num, speed_mode, channel, timer_sel, duty, hpoint;
};
struct ledc_timer_config_t  {
	int speed_mode,          // timer mode
	duty_resolution, // resolution of PWM duty
	timer_num,          // timer index
	freq_hz, 
	clk_cfg;
};

typedef int ledc_timer_t;
typedef int ledc_mode_t;
typedef int ledc_channel_t;

#define LEDC_CHANNEL_2 2
#define LEDC_TIMER_0 0 
#define LEDC_LOW_SPEED_MODE 0
#define LEDC_TIMER_6_BIT 0 
#define LEDC_USE_RTC8M_CLK 0 

#define CONFIG_CONSOLE_UART_NUM  0
static inline void uart_tx_wait_idle(int) {}
static inline void esp_sleep_pd_config(int, int) {}
#define ESP_PD_DOMAIN_RTC_PERIPH 0
#define ESP_PD_OPTION_AUTO 0 
extern int Csim_currentPwm[];

static inline void ledc_update_duty(int, int) {}
//#define LEDC_LS_MODE 0
static inline void ledc_set_duty(int, int chan, int val) {
	Csim_currentPwm[chan] = val;
}
static inline int ledc_get_duty(int, int) { return 0; }
static inline void ledc_timer_config(void *) {}
static inline void ledc_channel_config(void *) {}

// simple pin manager to blindly return values with digitalRead 
// that were previously written with digitalWrite
class Csim_pinManager : public Csim_Module {
public:
	struct PressInfo { int pin; float start; float duration; };
	vector<PressInfo> presses;
	void add(int pin, float start, float duration) {
		PressInfo pi; 
		pi.pin = pin; pi.start = start; pi.duration = duration;
		presses.push_back(pi);
	}
	void addPress(int pin, float time, int clicks, bool longPress)  {
		for(int n = 0; n < clicks; n++) {
			float duration = longPress ? 2.5 : .2; 
			add(pin, time,  duration);
			time += duration + .2;
		}
	}
	Csim_pinManager() { 
		for(int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
			pins[i] = 1;
		setPinManager(this); 
	}
	int pins[128];
	virtual int analogRead(int p) { 
		return pins[p]; 
	}
	void csim_analogSet(int p, int v) {
		pins[p] = v;
	}
	virtual void digitalWrite(int p, int v) { pins[p] = v; }
	static Csim_pinManager *manager;
	static void setPinManager(Csim_pinManager *p) { manager = p; }  
	int digitalRead(int pin) {
		float now = millis() / 1000.0; // TODO this is kinda slow 
		for (vector<PressInfo>::iterator it = presses.begin(); it != presses.end(); it++) { 
			if (it->pin == pin && now >= it->start && now < it->start + it->duration)
				return 0;
		} 
		return pins[pin];
	}
}; 

// Takes an input of a text file with line-delimited usec intervals between 
// interrupts, delivers an interrupt to the sketch-provided ISR
// TODO: only handles one ISR and one interrupt source 
class InterruptManager { 
	ifstream ifile;
	uint64_t nextInt = 0; 
	int count = 0;
public:
	void (*intFunc)() = NULL;
	void getNext() { 
		int delta;
		ifile >> delta;
		delta = min(100000, delta);
		nextInt += delta;
		ifile >> delta;
		delta = min(50000, delta);
		nextInt += delta;
		count++;
	}
	void setInterruptFile(char const *fn) { 
		ifile = ifstream(fn, ios_base::in);
		getNext();
	}
	void run() {
		if (intFunc != NULL && ifile && micros() >= nextInt) { 
			intFunc();
			getNext();
		}
	}
};

extern InterruptManager intMan;

static inline void pinMode(int, int) {}
static inline void digitalWrite(int p, int v) { Csim_pinManager::manager->digitalWrite(p, v); };
static inline int digitalRead(int p) { return Csim_pinManager::manager->digitalRead(p); }
static inline int digitalPinToInterrupt(int) { return 0; }
static inline void attachInterrupt(int, void (*i)(), int) { intMan.intFunc = i; } 
static inline void ledcSetup(int, int, int) {}
static inline void ledcAttachPin(int, int) {}
#ifndef ESP32CORE_V2
static inline void ledcAttachChannel(int, int, int, int) {}
#endif
static inline void ledcDetachPin(int) {}
void delayMicroseconds(int m);
static inline void delay(int m) { delayMicroseconds(m*1000); }
static inline void yield() { intMan.run(); }
//void analogSetCycles(int) {}
static inline void adcAttachPin(int) {}
static inline int analogRead(int p) { return Csim_pinManager::manager->analogRead(p); } 
static inline void csim_analogSet(int p, int v) { Csim_pinManager::manager->csim_analogSet(p, v); }

#define radians(x) ((x)*M_PI/180)
#define degrees(x) ((x)*180.0/M_PI)
#define sq(x) ((x)*(x))
#define TWO_PI (2*M_PI)
#define INPUT_PULLUP 0 
#define OUTPUT 0 
#define CHANGE 0 
#define RISING 0
#define FALLING 0
#define INPUT 0 
#define INPUT_PULLDOWN 0
#define SERIAL_8N1 0
#define ST7735_RED 0 
#define ST7735_GREEN 0 
#define ST7735_BLACK 0 
#define ST7735_WHITE 0 
#define ST7735_YELLOW 0 

// stub out String.h
class String {
	public:
	string st;
	String(const char *s) : st(s) {}
	String(string s) : st(s) {}
	String(const char *b, int l) { 
		st = string();
		for (int n = 0; n < l; n++) {
			st.push_back(b[n]);
		}
	} 
	String(int s) : st(to_string(s)) {}
	String() {}
	String(int, int) {}
	int length() const { return st.length(); } 
	bool operator!=(const String& x) const { return st != x.st; }
	bool operator==(const String& x) const { return st == x.st; } 
	String operator+(const String& x) { return st + x.st; } 
	String operator+(const char *x) { return st + x; } 
	String operator+(char x) { return st + x; } 
	String &operator+=(char x) { st = st + x; return *this; } 
	String &operator+=(const char *x) { st = st + x; return *this; } 
	const char *c_str(void) const { return st.c_str(); }
	operator const char *() { return c_str(); }
	operator std::string() { return std::string(c_str()); } 
	int indexOf(char c) { return st.find(c); } 
	String substring(int a, int b) { return String(st.substr(a, b)); }  
	void replace(const char *, const char *) {}
};

static inline String operator +(const char *a, const String &b) { return String(a) + b; }
static inline String operator +(const String &a, const char *b) { return String(a) + String(b); }
static inline bool operator ==(const char *a, const String &b) { return b.st == a; }
static inline bool operator ==(const String &a, const char *b) { return a.st == b; }
static inline bool operator ==(const String &a, const String &b) { return a.st == b.st; }

class IPAddress {
public:
	IPAddress(int, int, int, int) {}
	IPAddress() {}
	void fromString(const char *) {}
	String toString() const { return String("10.0.0.1"); }	
    int operator [](int) { return 0; }  
};

// stub out Serial.h
class FakeSerial { 
public:
	deque<pair<uint64_t, String>> inputQueue;
	String inputLine;
	int toConsole = 0;
	void begin(int a = 0, int b = 0, int c = 0, int d = 0, bool e = 0) {}
	void print(int, int) {}
	void print(const char *p) { this->printf("%s", p); }
	void println(const char *p= NULL) { this->printf("%s\n", p != NULL ? p : ""); }
	void print(char c) { this->printf("%c", c); } 
	void printf(const char  *f, ...);
	void setTimeout(int) {}
	void flush() {} 
	int availableForWrite() { return 1; } 
	int available();
	int readBytes(uint8_t *b, int l);
	int write(const uint8_t *, int) { return 0; }
	int write(const char *) { return 0; }	
	int read(char * b, int l) { return readBytes((uint8_t *)b, l); }
	int read();
	void scheduleInput(int64_t ms, const String &s) { inputQueue.push_back({ms, s});}
};
typedef FakeSerial Stream;
extern FakeSerial Serial, Serial1, Serial2;

// stub out WiFi.h
struct WiFiManager {
};

#define WL_DISCONNECTED 0
#define WL_CONNECTED 1
#define WIFI_STA 0
#define WIFI_OFF 0
#define DEC 0
#define HEX 0 

#include <arpa/inet.h>
#if __BIG_ENDIAN__
# define htonll(x) (x)
# define ntohll(x) (x)
#else
# define htonll(x) (((uint64_t)htonl((x) & 0xFFFFFFFF) << 32) | htonl((x) >> 32))
# define ntohll(x) (((uint64_t)ntohl((x) & 0xFFFFFFFF) << 32) | ntohl((x) >> 32))
#endif
	
void esp_read_mac(uint8_t *out, int);

typedef int gpio_num_t;

inline static void gpio_deep_sleep_hold_dis() {}
inline static void gpio_deep_sleep_hold_dis(int) {}
inline static void gpio_deep_sleep_hold_en() {}
inline static void gpio_hold_dis(int)  {}
inline static void gpio_hold_en(int)  {}
extern uint64_t sleep_timer;
inline static int esp_sleep_enable_timer_wakeup(uint64_t t) { sleep_timer = t; return 0; }

// TODO move all this to csim object
typedef std::function<void(uint64_t usec)> deepSleepHookT;
void csim_onDeepSleep(deepSleepHookT func);
void esp_deep_sleep_start();
void esp_light_sleep_start();

// stub out ArduinoJson library
struct JsonResult { 
	operator int()  { return 0; }
	operator const char *() { return ""; }	
};
template<int N>
struct StaticJsonDocument { 
	JsonResult operator[] (const char *) { return JsonResult(); }
};

//typedef int DeserializationError;
//int deserializeJson(StaticJsonDocument<1024>, String) { return 0; }

struct FakeWiFi {
	int simulatedFailMinutes = 0;  /* if n > 0 fail for a minute every nth minute */
	int curStatus = WL_DISCONNECTED;
	int begin(const char *, const char *) { curStatus = WL_CONNECTED; return 0; }
	int status();
	IPAddress localIP() { return IPAddress(); } 
	void setSleep(bool) {}
	void mode(int) {}
	int isConnected() { return status() == WL_CONNECTED; } 
	int channel() { return 0; }
	void disconnect(bool) {}
	void scanDelete() {}
	String SSID(int i = 0) { return String("CSIM"); }
	int waitForConnectResult() { return 0; }
	int scanNetworks() { return 0; }
	int RSSI(int i = 0) { return 63; }
	void disconnect() {}
	void reconnect() {}
	String macAddress() { return String("DEADBEEFFF"); }
};

extern FakeWiFi WiFi;

class WiFiClientSecure {
	public:
	void setInsecure() {}
};
 
struct WiFiClient { 
	string buffer;
	int available() { return buffer.length(); }
	int readBytes(uint8_t *buf, int len) {
		int n = min(len, (int)buffer.length());
		memcpy(buf, buffer.c_str(), n);
		buffer.erase(0, n);
		return n;
	}
	int connected() { return 1; }
	void setTimeout(int) {}
	void stop() {}
	void flush() {}
	int write(const uint8_t *, int) { return 0; }
	int read(uint8_t *buf, int len) { return this->readBytes(buf, len); }
	int connect(const char *, int, int tmo = 0) { return 0; } 
};

struct WiFiServer { 
	int begin(int) { return 0; }
	WiFiClient client;
	WiFiClient available() { return client; }
};

class HTTPClient { 
	string header1, header2, response, url;
	WiFiClient wc;
public:
	void setTimeout(int) {}
	void setConnectTimeout(int) {}
    int begin(const char *url) { this->url = url; return 0; }
	int begin(WiFiClientSecure, const char *) { return 0; }
	String getString() { return String(response.c_str()); }
	int GET() { 
		return csim_doPOSTorGET(false, "", wc.buffer);
	}
	int getSize() { return wc.buffer.length(); }
	WiFiClient *getStreamPtr() { return &wc; } 
	bool connected() { return true; } 
	void end() {}
	void addHeader(const char *h1, const char *h2) { header1 = h1; header2 = h2; }
	int POST(const char *postData) {
		return csim_doPOSTorGET(true, postData, response);
	}
private:
	typedef std::function<int(const char *url, const char *hdr, const char *data, string &result)> postHookT;
	struct postHookInfo {
		string urlRegex;
		bool isPost; // otherwise its a get
		postHookT func;
	};
	static vector<postHookInfo> &csim_hooks();
	int csim_doPOSTorGET(bool isPost, const char *data, string &result);
public:
	static void csim_onPOST(const string &url, postHookT func) {
		csim_hooks().push_back({url, true, func});
	}
	static void csim_onGET(const string &url, postHookT func) {
		csim_hooks().push_back({url, false, func});
	}
};

// stub out PubSubClient library
class PubSubClient : public Csim_Module {
	std::function<void(char *, byte *p, unsigned int)> callback = nullptr;
	struct Event {
		float sec;
		String t, p;
	};
	vector<Event> events;
  public:
	void parseArg(char **&a, char **) override {
		if (strcmp(*a, "--mqtt") == 0) {
			Event e; 
			sscanf(*(++a), "%f", &e.sec);
			e.t = String(*(++a));
			e.p = String(*(++a));
			events.push_back(e);
		}
	}
	virtual void loop() override { 
		for(vector<Event>::iterator i = events.begin(); i != events.end(); i++) {
			if (i->sec < _micros / 1000000.0) { 
				if (callback) { 
					callback((char *)i->t.c_str(), (byte *)i->p.c_str(), i->p.length());
				}
				events.erase(i);
				break;
			}
		}
	}
	int isConnected = 0;
	PubSubClient(WiFiClient &) {}
	void publish(const char *t, const char *p, int l = 0)  {
		//printf("MQTT: %s: %s\n", t, p);
	}
	int connected() { return isConnected; }
	int connect(const char *) { return isConnected = 1; }
	int subscribe(const char *) { return 1; }
	void setServer(const char *, int) {}
	String state() { return String(); }
	int setCallback(std::function<void(char *, byte *p, unsigned int)> c) { 
		callback = c;
		return 0; 
	} 
};

// stub out SD library
class FakeSD {
	public:
	bool begin(int, int, int, int) { return true; }
	fs::File open(const char *) { return fs::File(); } 
};
extern FakeSD SD;

class WiFiMulti {
public:
	void addAP(const char *, const char *) {}
	void run() {}
};

class WiFiUDP {
	int port, txPort;
	bool toSerial = false;
public:
	void begin(int p) { port = p; }
	int beginPacket(IPAddress, int p) { return beginPacket(NULL, p); }
	int beginPacket(const char *, int p) { txPort = p; return 1; }
	void write(const uint8_t *b, int len) {
		float f;
		if (txPort == 7892 && sscanf((const char *)b, "trim %f", &f) == 1) {
			//Csim_pitchCmd = f;
		}
	}
	int endPacket() { return 1; }

	typedef vector<unsigned char> InputData;
	typedef map<int, InputData> InputMap;
	static InputMap inputMap;
	int  parsePacket() { 
		if (inputMap.find(port) != inputMap.end()) { 
			return inputMap.find(port)->second.size();
		} else { 
			return 0;
		}
	}
	int read(uint8_t *b, int l) {
		if (inputMap.find(port) != inputMap.end()) { 
			InputData in = inputMap.find(port)->second;
			int rval = min((int)in.size(), l);
			for (int n = 0; n < rval; n++) { 
				b[n] = in.at(n);
			}
			if (toSerial) { 
				printf("UDP: %s", b);
			}
			inputMap.erase(port);
			return rval;
		} else { 
			return 0;
		}
	}
	IPAddress remoteIP() { return IPAddress(); } 
};

// stub out Pinger library
struct PingerResponse {
	int ReceivedResponse = 0;
};
typedef std::function<bool (const PingerResponse &)>PingerCallback;
struct FakePinger { 
  void OnReceive(PingerCallback) {}
  void Ping(IPAddress, int, int) {}
};

typedef FakePinger Pinger;

static inline void Csim_udpInput(int p, const WiFiUDP::InputData &s) { 
	WiFiUDP::InputMap &m = WiFiUDP::inputMap;
	if (m.find(p) == m.end())
		m[p] = s;
	else
		m[p].insert(m[p].end(), s.begin(), s.end());
}

static inline void Csim_udpInput(int p, const string &s) {
	Csim_udpInput(p, WiFiUDP::InputData(s.begin(), s.end()));
} 

// stub out NTPClient library
struct NTPClient {
	NTPClient(WiFiUDP &a) {}
	void update() {}
	void begin() {}
	long getEpochTime() { return 50000000 + millis() / 1000.0; }
	int getMinutes() { return millis() % (60 * 60 * 1000) / (60 * 1000); }
	int getHours() { return millis() % (60 * 60 * 24 * 1000) / (60 * 60 * 1000); }
	int getSeconds() { return millis() % (60 * 1000) / (1000); }
	void setUpdateInterval(int) {}
	String getFormattedTime() { return String("TIMESTRING"); }
};

// stub out Wire.h
class FakeWire {
public:
	void begin(int, int) {}
	void beginTransmission(int) {}
	bool endTransmission() { return false; }
};
extern FakeWire Wire;

#define ESP_MAC_WIFI_STA 0
#define ESP_OK 0 
#define WIFI_PHY_RATE_24M 0 
#define ESP_IF_WIFI_STA 0 
#define WIFI_SECOND_CHAN_NONE 0 
#define WIFI_IF_AP 0

typedef enum {
    ESP_NOW_SEND_SUCCESS = 0,       /**< Send ESPNOW data successfully */
    ESP_NOW_SEND_FAIL,              /**< Send ESPNOW data fail */
} esp_now_send_status_t;

typedef struct {
	uint8_t peer_addr[6];
	bool encrypt; 
	int channel;	
} esp_now_peer_info_t;

typedef struct { 
	int ampdu_tx_enable;
} wifi_init_config_t;

#define WIFI_INIT_CONFIG_DEFAULT() {0}
typedef struct { uint8_t *src_addr; } esp_now_recv_info;
typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr, const uint8_t *data, int data_len);
typedef void (*esp_now_recv_cb_t_v3)(const esp_now_recv_info *info, const uint8_t *data, int data_len);
typedef void (*esp_now_send_cb_t)(const uint8_t *mac_addr, esp_now_send_status_t status);

class ESPNOW_csimInterface {
public:
	virtual void send(const uint8_t *mac_addr, const uint8_t *data, int data_len) = 0;
};

// ESPNOW_csimOneProg: lightweight ESPNOW simluation framework where 
// one sketch creates both a client and server object and calls them 
// both interleaved from the same loop() code.  Depends on typical client/server
// code reading before writing, otherwise the module will consume its own output
// 
// #ifdef CSIM
// if (csim_flags.OneProg) { 
//	client1.run();
//	client2.run();
//	server.run();
//}
//#endif

class ESPNOW_csimOneProg : public Csim_Module, public ESPNOW_csimInterface {
	struct SimPacket {
		uint8_t mac[6];
		string data;
	};
	vector<SimPacket> pktQueue;
public:
	ESPNOW_csimOneProg() {}
	void send(const uint8_t *mac_addr, const uint8_t *data, int data_len) override;
	virtual void loop() override;
};

extern ESPNOW_csimInterface *ESPNOW_sendHandler;
extern esp_now_send_cb_t ESP32_esp_now_send_cb;
extern esp_now_recv_cb_t ESP32_esp_now_recv_cb;

static inline int esp_wifi_internal_set_fix_rate(int, int, int) { return ESP_OK; } 
static inline int esp_now_register_recv_cb(void *) { return ESP_OK; }	
static inline int esp_now_register_send_cb( void *) { return ESP_OK; }
static inline int esp_now_init() { return ESP_OK; } 
static inline int esp_now_deinit() { return ESP_OK; } 
static inline int esp_now_add_peer(void *) { return ESP_OK; } 
static inline int esp_wifi_stop() { return ESP_OK; } 
static inline int esp_wifi_deinit() { return ESP_OK; } 
static inline int esp_wifi_init(wifi_init_config_t *) { return ESP_OK; } 
static inline int esp_wifi_start() { return ESP_OK; } 
static inline int esp_wifi_set_channel(int, int) { return ESP_OK; } 
static inline int esp_now_register_send_cb(esp_now_send_cb_t cb) { ESP32_esp_now_send_cb = cb; return ESP_OK; }
static inline int esp_now_register_recv_cb(esp_now_recv_cb_t cb) { ESP32_esp_now_recv_cb = cb; return ESP_OK; }
static inline int esp_now_register_recv_cb(esp_now_recv_cb_t_v3 cb) { return ESP_OK; }
static inline int esp_wifi_config_espnow_rate(int, int) { return ESP_OK; }
int esp_now_send(const uint8_t*mac, const uint8_t*data, size_t len);

#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 1
#define INV_XYZ_COMPASS 0
static const int ACC_FULL_SCALE_4_G = 0, GYRO_FULL_SCALE_250_DPS = 0, MAG_MODE_CONTINUOUS_100HZ = 0;

class MPU9250_DMP {
public:
	int address = 0;
	int begin(){ return 1; }
	void setWire(FakeWire *) {}
	void setGyroFSR(int) {};
    void setAccelFSR(int) {};
    void setSensors(int) {}
	void updateAccel() {}
	void accelUpdate() { updateAccel(); }
	void magUpdate() { updateCompass(); }
	void gyroUpdate() { updateGyro(); }

	float accelX() { return ax; } 
	float accelY() { return ay; } 
	float accelZ() { return az; } 
	float magX() { return mx; } 
	float magY() { return my; } 
	float magZ() { return mz; } 
	float gyroX() { return gx; } 
	float gyroY() { return gy; } 
	float gyroZ() { return gz; } 
	
	void beginAccel(int) { begin(); }
	void beginGyro(int) {}
	void beginMag(int) {}
	int readId(uint8_t *) { return 0; } 
	
	void updateGyro() {}
	void updateCompass() {}
	float calcAccel(float x) { return x; }
	float calcGyro(float x) { return x; }
	float calcQuat(float x) { return x; }
	float calcMag(float x) { return x; }
	float ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz;
	//pitch,roll,yaw;
	MPU9250_DMP(int addr = 0x68) { bzero(this, sizeof(this)); } 
};

typedef MPU9250_DMP MPU9250_asukiaaa;

#define UPLOAD_FILE_START 0 
#define UPLOAD_FILE_END 0 
#define UPLOAD_FILE_WRITE 0 
#define UPDATE_SIZE_UNKNOWN 0

struct Update_t {
	int fd;
	int hasError() { return 0; } 
	int write(const uint8_t *buf, int len) { 
		return ::write(fd, buf, len); 
	}
	bool begin(int) { 
		fd = open("update.bin", O_CREAT|O_WRONLY, 0644);
		return true; 
	} 
	void printError(FakeSerial &) {}
	bool end(int) { 
		close(fd);
		return true; 
	}
	const char *errorString() { return "Update error"; }
};

extern Update_t Update;

class HTTPUpload {
public:
	int status, currentSize, totalSize;
	String filename;
	const char *buf;
};

#define HTTP_GET 0 
#define HTTP_POST 0
#define U_FLASH 0  

class WebServer {
	HTTPUpload u;
	public:
	WebServer(int) {}
	void begin() {}
	void on(const char *, int, function<void(void)>, function<void(void)>) {}
	void on(const char *, int, function<void(void)>) {}
	void sendHeader(const char *, const char *) {}
	void send(int, const char *, const char *) {}
	HTTPUpload &upload() { return u; }
	void handleClient() {}
};

class FakeCAN {
	typedef std::function<void(int)> callbackT;
	callbackT callback = NULL;
	ifstream simFile;
	string simFileLine;
	int packetByteIndex = 0;
	int packetLen = 0;
	int packetAddr;
	float firstPacketTs = 0;
	float pendingPacketTs = 0;
public:
	FakeCAN() {}
	int begin(long baudRate) { return 1; }
	void end() {}
	int endPacket() { return 0; }
	int parsePacket() { return 0; }
	int packetId() { return packetAddr; }
	int read();
	int packetRtr()  { return 0; }
	void onReceive(callbackT cb) { callback = cb; }
	int filter(int id, int mask) { return 0; }
	int filterExtended(long id, long mask) { return 0; }
	int setPins(int, int) { return 0; }
	int write(int) { return 0; } 
	int beginExtendedPacket(int) { return 0; } 
	// simulation hooks
	void setSimFile(const char *fn) { simFile.open(fn); }
	void run();  
};

extern FakeCAN CAN;

struct RTC_DS3231 {
};

#define COM_TYPE_UBX 0
class SFE_UBLOX_GPS {
public:
	double lat, lon;
	float hdg, hac, gs, siv, alt;
	bool fresh = false; 
	bool begin(FakeSerial &) { return true; } 
	bool enableDebugging(FakeSerial &, int) { return true; } 
	bool setUART1Output(int) { return true; } 
	float getHeading() { return hdg; }
	float getHeadingAccEst() { return hac; }
	double getLatitude() { return lat; }
	double getLongitude() { return lon; }
	float getAltitudeMSL() { return alt; }
	float getGroundSpeed() { return gs; }
	float getSIV() { return siv; }
	bool getPVT(int) { 
		bool rval = fresh;
		fresh = false;
		return rval;
	}	
	bool setSerialRate(int) { return 0; }
	bool setAutoPVT(int, int, int) { return 0; }
	void saveConfiguration() {}
	bool setNavigationFrequency(int) { return 0; } 
	bool getGnssFixOk() { return true; }
};

#define DHT22 0
struct sensors_event_t {
	float temperature = -1;
	float relative_humidity = -1;
};

struct DHT_Unified {
	struct response { void getEvent(sensors_event_t *) {} } resp;
	DHT_Unified(int, int) {}  
	int begin() { return 0; } 
	struct response temperature() { return resp; }
	struct response humidity() { return resp; } 
};

struct DHT {
	struct CsimInterface { 
		map<int,float> temp, humidity;
		void set(int pin, float t, float h) { temp[pin] = t; humidity[pin] = h; }
	};
	static CsimInterface &csim();
	static void csim_set(int pin, float t, float h) { csim().set(pin, t, h); }
	int pin;
    DHT(int p , int) : pin(p) {} // static init order disaster { csim.set(pin, 22.22, 77.77); } 
	void begin() {}
    float readTemperature(bool t = false, bool f = false) { return csim().temp[pin]; }
    float readHumidity(bool f = false) { return csim().humidity[pin]; }
};

class SimulatedFailureManager {
	struct FailSpec;
	vector<FailSpec> failList;
public:
	SimulatedFailureManager();
	void addFailure(const string &spec);
	void addFailure(const string &name, float chance, float duty, float period, float start=0);
	bool fail(const string &name);
};

SimulatedFailureManager &simFailures();
#define SIMFAILURE(x) simFailures().fail(x)

#endif // #ifdef _ESP32SIM_UBUNTU_H_
