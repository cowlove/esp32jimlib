#ifndef _ESP32SIM_UBUNTU_H_
#define _ESP32SIM_UBUNTU_H_
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


#ifndef GIT_VERSION
#define GIT_VERSION "no-git-version"
#endif
struct sensors_event_t {
	float temperature = -1;
	float relative_humidity = -1;
};

#define DHT22 0
struct DHT_Unified {
	struct response { void getEvent(sensors_event_t *) {} } resp;
	DHT_Unified(int, int) {}  
	int begin() { return 0; } 
	struct response temperature() { return resp; }
	struct response humidity() { return resp; } 
};

class ESP32sim_Module {
public:
	ESP32sim_Module();
	virtual void parseArg( char **&, char **) {}
	virtual void setup() {};
	virtual void loop() {};
	virtual void done() {};
};

#define byte char
static uint64_t _micros = 0;
static uint64_t _microsMax = 0xffffffff;
uint64_t micros() { return _microsMax > 0 ? ++_micros & _microsMax : ++_micros; }
uint64_t millis() { return ++_micros / 1000; }
void ESP32sim_exit(); 

// Stub out FreeRTOS stuff 
typedef int SemaphoreHandle_t;
static int Semaphores[10];
static int nextSem = 0;
int xSemaphoreCreateCounting(int max, int init) { 
	Semaphores[nextSem] = init;
	return nextSem++;
} 
int xSemaphoreCreateMutex() { return xSemaphoreCreateCounting(1, 1); } 
int xSemaphoreGive(int h) { Semaphores[h]++; return 0; } 
int xSemaphoreTake(int h, int delay) {
	if (Semaphores[h] > 0) {
		Semaphores[h]--;
		return 1;
	}
	return 0;
}
int uxSemaphoreGetCount(int h) { return Semaphores[h]; }



#define portMAX_DELAY 0 
#define tskIDLE_PRIORITY 0
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 0 	
void xTaskCreate(void (*)(void *), const char *, int, void *, int, void *) {}
#define WRITE_PERI_REG(a, b) if(0) {}
#define RTC_CNTL_BROWN_OUT_REG 0

typedef int esp_err_t;
typedef struct { int timeout_ms, idle_core_mask, trigger_panic; } esp_task_wdt_config_t;  
void esp_task_wdt_init(const esp_task_wdt_config_t *) {}
void esp_task_wdt_init(int, int) {}
void esp_task_wdt_reset() {}
esp_err_t esp_task_wdt_add(void *) { return 0; }
esp_err_t esp_task_wdt_delete(const void *) { return 0; }
int rtc_get_reset_reason(int) { return 0; } 
//static inline void ledcSetup(int, int, int) {}
//static inline void ledcAttachPin(int, int) {}
//static inline void ledcWrite(int, int) {}

#define ADC1_CHANNEL_1 0
#define ADC1_CHANNEL_2 0
#define ADC1_CHANNEL_3 0
int adc1_get_raw(int) { return 0; }
#define delayUs(x) delayMicroseconds(x)
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


namespace fs { 
class File {
	public: 
	bool operator!() { return false; } 
	operator bool() { return false; } 
	File openNextFile(void) { return *this; }
	void close() {}
        int print(const char *) { return 0; }
	int printf(const char *, ...) { return 0; } 
	int write(const char *, int) { return 0; } 
	int write(const uint8_t *, int) { return 0; } 
	int flush() { return 0; }	
	int read(uint8_t *, int) { return 0; } 
};
};
using fs::File;

struct FakeSPIFFS {
	void begin() {}
	void format() {}
	File open(const char *, const char *) { return File(); } 
} SPIFFS;

struct FakeArduinoOTA {
	void begin() {}
	void handle() {}
	int getCommand() { return 0; } 
	void onEnd(function<void(void)>) {}
	void onStart(function<void(void)>) {}
	void onError(function<void(int)>) {}
	void onProgress(function<void(int, int)>) {}
} ArduinoOTA;

typedef int ota_error_t; 
#define OTA_AUTH_ERROR 0 
#define OTA_BEGIN_ERROR 0 
#define OTA_CONNECT_ERROR 0 
#define OTA_RECEIVE_ERROR 0 
#define OTA_END_ERROR 0 

struct FakeESP {
	int getFreeHeap() { return 0; }
	void restart() {}
	int getChipId() { return 0xdeadbeef; }
} ESP;

struct WiFiManager {
};

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



static int ESP32sim_currentPwm[16];
void ledcWrite(int chan, int val) {
		ESP32sim_currentPwm[chan] = val;
} 

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
} intMan;


// simple pin manager to simply return values that were written
class ESP32sim_pinManager : public ESP32sim_Module {
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
	ESP32sim_pinManager() { 
		for(int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
			pins[i] = 1;
		setPinManager(this); 
	}
	int pins[128];
	virtual int analogRead(int p) { return 1; }
	virtual void digitalWrite(int p, int v) { pins[p] = v; }
	static ESP32sim_pinManager *manager;
	static void setPinManager(ESP32sim_pinManager *p) { manager = p; }  
	int digitalRead(int pin) {
		float now = millis() / 1000.0; // TODO this is kinda slow 
		for (vector<PressInfo>::iterator it = presses.begin(); it != presses.end(); it++) { 
			if (it->pin == pin && now >= it->start && now < it->start + it->duration)
				return 0;
		} 
		return pins[pin];
	}
} pinManObject;

ESP32sim_pinManager *ESP32sim_pinManager::manager = &pinManObject;

void pinMode(int, int) {}
void digitalWrite(int p, int v) { ESP32sim_pinManager::manager->digitalWrite(p, v); };
int digitalRead(int p) { return ESP32sim_pinManager::manager->digitalRead(p); }
int digitalPinToInterrupt(int) { return 0; }
void attachInterrupt(int, void (*i)(), int) { intMan.intFunc = i; } 
void ledcSetup(int, int, int) {}
void ledcAttachPin(int, int) {}
#ifndef ESP32CORE_V2
void ledcAttachChannel(int, int, int, int) {}
#endif
void ledcDetachPin(int) {}
void delayMicroseconds(int m) { _micros += m; intMan.run(); }
void delay(int m) { delayMicroseconds(m*1000); }
void yield() { intMan.run(); }
//void analogSetCycles(int) {}
void adcAttachPin(int) {}
int analogRead(int p) { return ESP32sim_pinManager::manager->analogRead(p); } 

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

String operator +(const char *a, const String &b) { return String(a) + b; }
String operator +(const String &a, const char *b) { return String(a) + String(b); }
bool operator ==(const char *a, const String &b) { return b.st == a; }
bool operator ==(const String &a, const char *b) { return a.st == b; }
bool operator ==(const String &a, const String &b) { return a.st == b.st; }
class IPAddress {
public:
	IPAddress(int, int, int, int) {}
	IPAddress() {}
	void fromString(const char *) {}
	String toString() const { return String("0.0.0.0"); }	
    int operator [](int) { return 0; }  
};

class FakeSerial { 
	public:
	deque<pair<uint64_t,String>> inputQueue;
	String inputLine;
	int toConsole = 0;
	void begin(int a = 0, int b = 0, int c = 0, int d = 0, bool e = 0) {}
	void print(int, int) {}
	void print(const char *p) { this->printf("%s", p); }
	void println(const char *p= NULL) { this->printf("%s\n", p != NULL ? p : ""); }
	void print(char c) { this->printf("%c", c); } 
	void printf(const char  *f, ...) { 
		va_list args;
		va_start (args, f);
		if (toConsole) 
			vprintf (f, args);
		va_end (args);
	}
	void setTimeout(int) {}
	void flush() {} 
	int availableForWrite() { return 1; } 
	int available() {
		if (inputLine.length() > 0)
			return inputLine.length(); 
		if (inputQueue.size() > 0 && millis() >= inputQueue[0].first) 
			return inputQueue[0].second.length();
		return 0;
	}
	int readBytes(uint8_t *b, int l) {
		if (inputLine.length() == 0 && inputQueue.size() > 0 && millis() >= inputQueue[0].first) { 
			inputLine = inputQueue[0].second;
			inputQueue.pop_front();
		}
		int rval =  min(inputLine.length(), l);
		if (rval > 0) {
			strncpy((char *)b, inputLine.c_str(), rval);
			if (rval < inputLine.length())
				inputLine = inputLine.substring(rval, -1);
			else 
				inputLine = "";
		}
		return rval;
	} 
	int write(const uint8_t *, int) { return 0; }
	int write(const char *) { return 0; }	
	int read(char * b, int l) { 
		return readBytes((uint8_t *)b, l);
	}
	int read() { 
		uint8_t b;
		if (readBytes(&b, 1) == 1)
			return b;
		return -1; 
	}

	void scheduleInput(int64_t ms, const String &s) { 
		inputQueue.push_back(pair<int64_t,String>(ms, s));
	}
} Serial, Serial1, Serial2;

typedef FakeSerial Stream;

#define WL_DISCONNECTED 0
#define WL_CONNECTED 1
#define WIFI_STA 0
#define WIFI_OFF 0
#define DEC 0
#define HEX 0 


	
void esp_read_mac(uint8_t *, int) {}




typedef int gpio_num_t;

void gpio_deep_sleep_hold_dis() {}
void gpio_deep_sleep_hold_dis(int) {}
void gpio_deep_sleep_hold_en() {}
void gpio_hold_dis(int)  {}
void gpio_hold_en(int)  {}
int esp_sleep_enable_timer_wakeup(uint64_t) { return 0; }
void esp_deep_sleep_start() { ESP32sim_exit(); }

struct JsonResult { 
	operator int()  { return 0; }
	operator const char *() { return ""; }	
};
template<int N>
struct StaticJsonDocument { 
	JsonResult operator[] (const char *) { return JsonResult(); }
};

typedef int DeserializationError;
int deserializeJson(StaticJsonDocument<1024>, String) { return 0; }

struct FakeWiFi {
	int curStatus = WL_DISCONNECTED;
	int begin(const char *, const char *) { curStatus = WL_CONNECTED; return 0; }
	int status() { return curStatus; } 
	IPAddress localIP() { return IPAddress(); } 
	void setSleep(bool) {}
	void mode(int) {}
	int isConnected() { return 0; }
	int channel() { return 0; }
	void disconnect(bool) {}
	void scanDelete() {}
	String SSID(int i = 0) { return String("FakeWiFi"); }
	int waitForConnectResult() { return 0; }
	int scanNetworks() { return 0; }
	String RSSI(int i = 0) { return String(); }
	void disconnect() {}
	void reconnect() {}
	String macAddress() { return String("DEADBEEFFF"); }
} WiFi;

class WiFiClientSecure {
	public:
	void setInsecure() {}
};
 
struct WiFiClient { 
	int available() { return 0; }
	int readBytes(uint8_t *, int) { return 0; }
	int connected() { return 0; }
	void setTimeout(int) {}
	void stop() {}
	void flush() {}
	int write(const uint8_t *, int) { return 0; }
	int read(uint8_t *, int) { return 0; }
	int connect(const char *, int, int tmo = 0) { return 0; } 
};

struct WiFiServer { 
	int begin(int) { return 0; }
	WiFiClient client;
	WiFiClient available() { return client; }
};

class HTTPClient { 
	public:
        int begin(const char *) { return 0; }
	int begin(WiFiClientSecure, const char *) { return 0; }
	String getString() { return String(); }
	int GET() { return 0; }
	int getSize() { return 0; }
	WiFiClient *getStreamPtr() { return 0; } 
	bool connected() { return 0; } 
	void end() {}
	void addHeader(const char *, const char *) {}
	int POST(const char *) { return -1; }
};

#define PROGMEM 




class PubSubClient : public ESP32sim_Module {
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
		printf("MQTT: %s: %s\n", t, p);
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
class FakeSD {
	public:
	bool begin(int, int, int, int) { return true; }
	fs::File open(const char *) { return fs::File(); } 
} SD;

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
			//ESP32sim_pitchCmd = f;
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

struct PingerResponse {
	int ReceivedResponse = 0;
};
typedef std::function<bool (const PingerResponse &)>PingerCallback;
struct FakePinger { 
  void OnReceive(PingerCallback) {}
  void Ping(IPAddress, int, int) {}
};

typedef FakePinger Pinger;

// TODO: extend this to use vector<unsigned char> to handle binary data	
WiFiUDP::InputMap WiFiUDP::inputMap;

void ESP32sim_udpInput(int p, const WiFiUDP::InputData &s) { 
	WiFiUDP::InputMap &m = WiFiUDP::inputMap;
	if (m.find(p) == m.end())
		m[p] = s;
	else
		m[p].insert(m[p].end(), s.begin(), s.end());
}

void ESP32sim_udpInput(int p, const string &s) {
	ESP32sim_udpInput(p, WiFiUDP::InputData(s.begin(), s.end()));
} 

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

class FakeWire {
public:
	void begin(int, int) {}
	void beginTransmission(int) {}
	bool endTransmission() { return false; }
} Wire;

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

typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr, const uint8_t *data, int data_len);
typedef void (*esp_now_send_cb_t)(const uint8_t *mac_addr, esp_now_send_status_t status);
esp_now_send_cb_t ESP32_esp_now_send_cb = NULL;
esp_now_recv_cb_t ESP32_esp_now_recv_cb = NULL, ESP32_esp_now_csim_send_handler = 0;

class ESPNOW_csim : public ESP32sim_Module {
	struct SimPacket {
		uint8_t mac[6];
		string data;
	 };
	vector<SimPacket> pktQueue;
public:
	static ESPNOW_csim *Instance;
	ESPNOW_csim() { Instance = this; }
	void send(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
		SimPacket p; 
		memcpy(p.mac, mac_addr, sizeof(p.mac));
		const char *cp = (const char *)data;
		p.data = string(cp, cp + data_len);
		pktQueue.push_back(p);
	}
	virtual void loop() override {
		for(auto pkt : pktQueue) {
			if (ESP32_esp_now_recv_cb != NULL) {
				ESP32_esp_now_recv_cb(pkt.mac, (const uint8_t *)pkt.data.c_str(), pkt.data.length());
			}
		}
		pktQueue.clear();
	} 
} espnow;
ESPNOW_csim *ESPNOW_csim::Instance = NULL;

int esp_wifi_internal_set_fix_rate(int, int, int) { return ESP_OK; } 
int esp_now_register_recv_cb(void *) { return ESP_OK; }	
int esp_now_register_send_cb( void *) { return ESP_OK; }
int esp_now_init() { return ESP_OK; } 
int esp_now_deinit() { return ESP_OK; } 
int esp_now_add_peer(void *) { return ESP_OK; } 
int esp_wifi_stop() { return ESP_OK; } 
int esp_wifi_deinit() { return ESP_OK; } 
int esp_wifi_init(wifi_init_config_t *) { return ESP_OK; } 
int esp_wifi_start() { return ESP_OK; } 
int esp_wifi_set_channel(int, int) { return ESP_OK; } 
int esp_now_register_send_cb(esp_now_send_cb_t cb) { ESP32_esp_now_send_cb = cb; return ESP_OK; }
int esp_now_register_recv_cb(esp_now_recv_cb_t cb) { ESP32_esp_now_recv_cb = cb; return ESP_OK; }
int esp_now_send(const uint8_t*mac, const uint8_t*data, size_t len) {
	if (ESP32_esp_now_send_cb != NULL)
		ESP32_esp_now_send_cb(mac, ESP_NOW_SEND_SUCCESS); 
	if (ESPNOW_csim::Instance != NULL)
		ESPNOW_csim::Instance->send(mac, data, len); 
	return ESP_OK; 
}
int esp_wifi_config_espnow_rate(int, int) { return ESP_OK; }

#define INV_SUCCESS 1
#define INV_XYZ_GYRO 1
#define INV_XYZ_ACCEL 1
#define INV_XYZ_COMPASS 0
const int ACC_FULL_SCALE_4_G = 0, GYRO_FULL_SCALE_250_DPS = 0, MAG_MODE_CONTINUOUS_100HZ = 0;

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

struct {
	int hasError() { return 0; } 
	int write(const uint8_t *, int) { return 0; }
	int begin(int) { return 0; } 
	void printError(FakeSerial &) {}
	int end(int) { return 0; }
} Update;

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
public:
  FakeCAN() {}
  int begin(long baudRate) { return 1; }
  void end() {}
  int endPacket() { return 0; }
  int parsePacket() { return 0; }
  int packetId() { return packetAddr; }
  int read() { 
	char *p = strchr((char *)simFileLine.c_str(), ']');
	if (p == NULL) return 0;
	for (int i = 0; i < packetByteIndex / 8 + 1; i++) {
		p = strchr(p + 1, ' ');
		if (p == NULL) return 0;
	}
	unsigned int rval;
	char b[3];
	b[0] = *(p + 1 + (packetByteIndex % 8) * 2);
	b[1] = *(p + 2 + (packetByteIndex % 8) * 2);
	b[3] = 0;
	if (sscanf(b, "%02x", &rval) != 1)
		return 0;
	packetByteIndex++;
	return rval; 
  }
  int packetRtr()  { return 0; }
  typedef void(*callbackT)(int);
  callbackT callback = NULL;
  void onReceive(callbackT cb) { callback = cb; }
  int filter(int id, int mask) { return 0; }
  int filterExtended(long id, long mask) { return 0; }
  int setPins(int, int) { return 0; }
  int write(int) { return 0; } 
  int beginExtendedPacket(int) { return 0; } 
// simulation hooks
  ifstream simFile;
  string simFileLine;
  int packetByteIndex = 0;
  int packetLen = 0;
  int packetAddr;
  float firstPacketTs = 0;
  float pendingPacketTs = 0;


  void setSimFile(const char *fn) { simFile.open(fn); }
  void run() { 
	if (callback == NULL || !simFile) 
		return;
	
	if (simFileLine.size() == 0) { 
		std::getline(simFile, simFileLine);
		if (sscanf(simFileLine.c_str(), " (%f) can0 %x [%d]", 
			&pendingPacketTs, &packetAddr, &packetLen) != 3) 
			return;
	}

	if (simFileLine.size() > 0) { 
		packetByteIndex = 0;
		if (firstPacketTs == 0) firstPacketTs = pendingPacketTs;
		if (micros() > (pendingPacketTs - firstPacketTs) * 1000000.0) {
			int remain = packetLen;
			while(remain > 0) { 
				int l = min(remain, 8);
				callback(l);
				remain -= 8;
			} 
			simFileLine = ""; 
		}
	}
  }
  
} CAN;

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


void setup(void);
void loop(void);

class ESP32sim {
public:
	vector<ESP32sim_Module *> modules;
	struct TimerInfo { 
		uint64_t last;
		uint64_t period;
		function<void(void)> func;
	};
	typedef vector<TimerInfo> timers;
	void main(int argc, char **argv) {
		float seconds = 0;
		Serial.toConsole = true;
		ESPNOW_csim espnow;
		for(char **a = argv + 1; a < argv+argc; a++) {
			if (strcmp(*a, "--serial") == 0) {
				printf("--serial is depricated, use --serialConsole\n");
				::exit(-1);
			}
			else if (strcmp(*a, "--serialConsole") == 0) sscanf(*(++a), "%d", &Serial.toConsole); 
			else if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%f", &seconds); 
			else if (strcmp(*a, "--interruptFile") == 0) { 
				intMan.setInterruptFile(*(++a));
			} else if (strcmp(*a, "--button") == 0) {
                        int pin, clicks = 1, longclick = 0;
                        float tim;
                        sscanf(*(++a), "%f,%d,%d,%d", &tim, &pin, &clicks, &longclick);
                        ESP32sim_pinManager::manager->addPress(pin, tim, clicks, longclick);
			} else if (strcmp(*a, "--serialInput") == 0) {
				float seconds;
				sscanf(*(++a), "%f", &seconds);
				Serial.scheduleInput(seconds * 1000, String(*(++a)) + "\n");
			
			} else if (strcmp(*a, "--serial2Input") == 0) {
				float seconds;
				sscanf(*(++a), "%f", &seconds);
				Serial2.scheduleInput(seconds * 1000, String(*(++a)) + "\n");
			} else for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) {
				(*it)->parseArg(a, argv + argc);
			}
		}
		
		for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
			(*it)->setup();
		setup();

		uint64_t lastMillis = 0;
		while(seconds <= 0 || _micros / 1000000.0 < seconds) {
			uint64_t now = millis();
			//for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
			for(auto it : modules) {
				it->loop();
			}
			loop();
			intMan.run();

			//if (floor(now / 1000) != floor(lastMillis / 1000)) { 
			//	ESP32sim_JDisplay_forceUpdate();	
			//}
			lastMillis = now;
		}
	}
	void exit() { 
		for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
			(*it)->done();	
		::exit(0);
	}
} esp32sim;

void ESP32sim_exit() { 	esp32sim.exit(); }

inline ESP32sim_Module::ESP32sim_Module() { 
	esp32sim.modules.push_back(this);
}

int main(int argc, char **argv) {
	esp32sim.main(argc, argv);
}

#endif // #ifdef _ESP32SIM_UBUNTU_H_
