#ifndef INC_JIMLIB_H
#define INC_JIMLIB_H
#include <functional>
#include <string>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <fcntl.h>
#include <utility>
#include <regex>

#ifndef CSIM
#include "ArduinoOTA.h"
#include "WiFiUdp.h"



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

#include <stdarg.h>
#define DEG2RAD(x) ((x)*M_PI/180)
#define RAD2DEG(x) ((x)*180/M_PI)

void wdtInit(int sec);
void wdtReset();
void ledcInit(int pin, int freq, int res, int channel); 

std::string strfmt(const char *format, ...);
String Sfmt(const char *format, ...);
std::string sfmt(const char *format, ...);


int scanI2c();
void printPins();

#ifdef ESP32
class FakeMutex {
	public:
	void lock() {}
	void unlock() {}
};

class FakeSemaphore { 
 public:
	FakeSemaphore(int max = 1, int init = 1) {}
	bool take(int delay = 0) { return true; } 
	void give() {}
};

class Mutex {
	SemaphoreHandle_t xSem;
 public:
	Mutex() { 
		xSem = xSemaphoreCreateCounting(1,1);
		unlock();
	}
	void lock() { xSemaphoreTake(xSem, portMAX_DELAY); } 
	void unlock() { xSemaphoreGive(xSem); }
};

class Semaphore { 
	SemaphoreHandle_t xSem;
 public:
	Semaphore(int max = 1, int init = 1) { 
		xSem = xSemaphoreCreateCounting(max, init);
	}
	bool take(int delay = portMAX_DELAY) { return xSemaphoreTake(xSem, delay); } 
	void give() { xSemaphoreGive(xSem); }
	int getCount() { return uxSemaphoreGetCount(xSem); } 
};

class ScopedMutex {
	Mutex *mutex; 
  public:
	ScopedMutex(Mutex &m) : mutex(&m) { mutex->lock(); } 
	ScopedMutex(FakeMutex &m) : mutex(NULL) {} 
	~ScopedMutex() { if (mutex != NULL) mutex->unlock(); } 
};
#endif

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

class DigitalDebounce {
	EggTimer timer;
	bool recentlyPressed;
	long startPress;
	int lastDuration;
	int lastVal;
public:
	int duration;
	int count;
	DigitalDebounce(int ms = 50) : timer(ms), recentlyPressed(false),count(0) {}
	bool checkOneshot(bool button) {
		bool rval = false; 
		if (button == true) {
			if (timer.tick() && lastVal == false) 
				recentlyPressed = false;
			rval = !recentlyPressed;
			if (rval) {
				count++;
				startPress = millis();
			}
			recentlyPressed = true;
			timer.reset();
			duration = max(1UL, millis() - startPress);
		} else {
			duration = 0;
			if (timer.tick()) 
				recentlyPressed = false;
		}
		lastVal = button;
		return rval;
	}
	int checkEndPress() {
		int rval = 0; 
		if (duration == 0 && lastDuration > 0) 
			rval = lastDuration;
		lastDuration = duration;
		return rval;
	}
};

#ifdef ESP32
class RotaryEncoder {
public:
	int pin1, pin3;
	DigitalDebounce a,b;
	int limMin, limMax;
	int value;
	bool wrap;
	int count = 0;
	unsigned long lastChange;

#ifndef CSIM
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	void IRAM_ATTR ISR() {	
		portENTER_CRITICAL_ISR(&(this->mux));
		check();
		portEXIT_CRITICAL_ISR(&(this->mux));
	}
	void begin(void (*ISR_callback)(void)) {
		attachInterrupt(digitalPinToInterrupt(pin1), ISR_callback, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pin3), ISR_callback, CHANGE);
	}
#else
	void begin(void *) {};
#endif
	
	RotaryEncoder(int p1, int p3, int debounce = 5) : a(debounce), b(debounce), pin1(p1), pin3(p3){
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin3, INPUT_PULLUP);
		limMin = 001;
		limMax = 360;
		value = limMin;
		wrap = true;
		lastChange = 0;
	}
	void setLimits(int mn, int mx, bool wrap) {
		limMin = mn;
		limMax = mx;
		this->wrap = wrap;
	}
	void change(int delta) {
		if (delta != 0) { 
			value += delta;
			if (value < limMin) value = wrap ? limMax : limMin;
			if (value > limMax) value = wrap ? limMin : limMax;
		}
	}
	void check() {
		unsigned long now = millis();
		//if (now - lastChange > 500)
		//	Serial.printf("\n\n");
		count++;
		int buta = !digitalRead(pin1);
		int butb = !digitalRead(pin3);
		int oa = a.checkOneshot(buta);
		int ob = b.checkOneshot(butb);
		int delta = 0;
		if (oa && !butb) 
			delta = -1;
		if (ob && !buta)
			delta = +1;
		//Serial.printf("%d/%d %d/%d %d\n", buta, oa, butb, ob, delta );
		
		if (delta != 0) {
			if(lastChange > 0 && now - lastChange < 20)
				delta *= 5;
			lastChange = now;
		}
		change(delta);
	}
};
#endif 

class DigitalButton { 
	DigitalDebounce deb;
	bool inverted;
public:
	int pin;
	int count;
	int mode;
	DigitalButton(int p, bool invert = true, int m = INPUT_PULLUP, int debounceMs = 5) : pin(p), mode(m), inverted(invert), deb(debounceMs) {
		//pinMode(pin, mode);
	}
	bool read() { 
		pinMode(pin, mode);
		bool in = digitalRead(pin);
		return inverted ? !in : in;
	}
	bool check() {
		bool in = read();
		bool rval = deb.checkOneshot(in);
		if (rval) count++;
		return rval;
	}
	int duration() {
		check();
		return deb.duration;
	}
	int checkEndPress() {
		check();
		return deb.checkEndPress();
	}
};

template <class T>
class Changed {
	T old;
	bool first, cas;
public:
	Changed(int changeAtStart = false) : first(true), cas(changeAtStart) {}
	bool changed(T n) { 
		bool r = first ? cas : (n != old);
		old = n;
		first = false;
		return r;
	}
};

class LongShortFilter { 	
	int longPressMs, resetMs;
	unsigned long lastEndTime;
public:
	bool wasLong;
	int last, wasDuration;
	int count, wasCount;
	int lastDuration;
	int events;
	bool countedLongPress;
	bool finishingLong = false;
	LongShortFilter(int longMs, int resetTimeMs) : longPressMs(longMs), resetMs(resetTimeMs) {
		last = events = lastDuration = count = wasCount = lastEndTime = 0;
		countedLongPress = false;
	}
	
	bool check(int l) { 
		unsigned long now = millis();
		last = l;
		int rval = false;
		if (last == 0) { 
			if (lastDuration > 0) {  // press just ended, button up 
				count++;
				lastEndTime = now;
				wasDuration = lastDuration;
				lastDuration = 0;
			}
			if (lastEndTime > 0 && now - lastEndTime >= resetMs) { // resetMS after last button press 
				if (!countedLongPress) {
					wasCount = count;
					rval = true;
				}
				lastEndTime = count = 0;
				countedLongPress = false;
			} 
		} else { 
			if (last >= longPressMs && !countedLongPress) { // long press just finished, button still down
				countedLongPress = true;
				lastDuration = wasDuration = last;
				wasCount = ++count;
				rval = true;
			} else {
				lastDuration = last;
			}
		}
		if (rval == true) {
			events++;
			wasLong = wasDuration >= longPressMs;
		}
		return false;
	}
	bool inProgress() { return ((last != 0) || (lastEndTime > 0 && millis() - lastEndTime < resetMs)) && (countedLongPress == false); }
	int inProgressCount() { return count + ((last != 0) ? 1 : 0); }
	Changed<int> eventCount;
	bool newEvent() { 
		return eventCount.changed(events);
	}
};

class DigitalButtonLongShort { 
	public:
	LongShortFilter filter;
	DigitalButton button;
	DigitalButtonLongShort(int p, int l = 1000, int d = 250) : filter(l, d), button(p) {}
	bool newEvent() { return filter.newEvent(); } 
	void run() { 
		filter.check(button.duration());
	}
	bool newEventR() { run(); return newEvent(); }
	int count() { return filter.wasCount; } 
	bool inProgress() { return filter.inProgress(); } 
	int inProgressCount() { return filter.count; } 
	int wasLong() { return filter.wasLong; } 
};

template<class T> 
class StaleData {
	uint64_t timeout, lastUpdate, lastChange;
	T value, invalidValue;
	bool chg = false; 
public:
	StaleData(int t, T i) : lastUpdate(0), timeout(t), invalidValue(i) {} 
	bool isValid() { return millis() - lastUpdate < timeout; }
	operator T&() { return getValue(); }
	StaleData<T>& operator =(const T&v) {
		chg = value != v;
		value = v;
		lastUpdate = millis();
		if (chg) 
			lastChange = millis();
		return *this;
	}
	T& getValue() { return isValid() ? value : invalidValue; }
	bool changed() { 
		bool rval = chg;
		chg = false;
		return rval && isValid(); 
	}
	uint64_t age() { 
		return millis() - lastUpdate;
	}
	uint64_t timeSinceChange() { 
		return millis() - lastChange;
	}
};

template<class T> 
class ChangedData {
	T value;
	bool chg = false;
	bool first = true;
public:
	ChangedData(T i) {} 
	operator T&() { return value; }
	ChangedData<T>& operator =(const T&v) {
		chg = value != v || first;
		value = v;
		first = false;
		return *this;
	}
	bool changed() { 
		bool rval = chg;
		chg = false;
		return rval; }
};

#ifdef ESP32
template <typename Out>
inline void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}

static inline std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}


template <class T>  
class CircularBoundedQueue { 
	Semaphore empty, full;
	int size, head, tail;
	T *array;
public:
	CircularBoundedQueue(int s) : size(s), empty(s, s), full(s, 0) {
		array = new T[size];
		head = tail = 0;
	}
	T *peekHead(int tmo) {
		if (!empty.take(tmo)) 
			return NULL;

		T *rval = &array[head];
		head = (head + 1) % size;
		return rval;
	}
	void postHead() {
		full.give();
	}
	T *peekTail(int tmo) {
		if (!full.take(tmo)) 
			return NULL;

		T *rval = &array[tail];
		tail = (tail + 1) % size;
		return rval;
	}
	void freeTail() {
		empty.give();
	}
	int getCount() { return full.getCount(); } 
};
#endif //#ifdef ESP32

// From data format described in web search "SL30 Installation Manual PDF" 
class SL30 {
public:
        std::string twoenc(unsigned char x) {
                char r[3];
                r[0] = (((x & 0xf0) >> 4) + 0x30);
                r[1] = (x & 0xf) + 0x30;
                r[2] = 0;
                return std::string(r);
        }
        int chksum(const std::string& r) {
                int sum = 0;
                const char* s = r.c_str();
                while (*s)
                        sum += *s++;
                return sum & 0xff;
        }
        void open() {
        }
        std::string pmrrv(const char *r) {
                return std::string("$PMRRV") + r + twoenc(chksum(r)) + "\r\n";
                //Serial2.write(s.c_str());
				//Serial.printf("G5: %s", s.c_str());
                //Serial.write(s.c_str());
        }
        std::string setCDI(double hd, double vd) {
                int flags = 0b11111010;
                hd *= 127 / 3;
                vd *= 127 / 3;
                return pmrrv((std::string("21") + twoenc(hd) + twoenc(vd) + twoenc(flags)).c_str());
        }
};

class PinPulse { 
public:
	int pin;
	uint64_t toggleTime = 0;
	PinPulse(int p, int initval = 0) : pin(p) { pinMode(p, OUTPUT); digitalWrite(p, initval); } 
	void  pulse(int v, int ms) { 
		toggleTime = ms > 0 ? millis() + ms: 0;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, v);
	}
	void run() { 
		if (toggleTime > 0 && millis() >= toggleTime) {
			toggleTime = 0;
			pinMode(pin, OUTPUT);
			digitalWrite(pin, !digitalRead(pin));
		}
	}
};

static inline std::string nmeaChecksum(const std::string &s) { 
	char check = 0;
	for (const char &c : s)  
		check ^= c;
	char buf[8];
	snprintf(buf, sizeof(buf), "*%02X\n", (int)check);
	return std::string("$") + s + std::string(buf);	
}


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

const String &getMacAddress();

using std::string;
template<class T> bool fromString(const string &s, T&v);
template<> inline bool fromString(const string &s, int &v) { return sscanf(s.c_str(), "%d", &v) == 1; }
template<> inline bool fromString(const string &s, float &v) { return sscanf(s.c_str(), "%f", &v) == 1; }
template<> inline bool fromString(const string &s, string &v) { v = s; return true; }

template<class T> string toString(const T&v);
template<> inline string toString(const int &v) { return sfmt("%d", v); }
template<> inline string toString(const float &v) { return sfmt("%f", v); }
template<> inline string toString(const string &s) { return s; }

#define LP() printf("%09.3f %s:%d\n", millis() / 1000.0, basename(__FILE__), __LINE__)
template<> inline bool fromString(const string &s, std::vector<string> &v) { 
	v = split(s, '\n');
	return true; 
}

template<> inline string toString(const std::vector<string> &v) { 
	string rval;
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
	static void begin();
};

template<class T> 
class SPIFFSVariableESP32 : public SPIFFSVariableESP32Base { 
	const T def;
	T val;
public:
	SPIFFSVariableESP32(const char *f, const T &d) : def(d), val(d) {
		filename = f;
		defaultStringValue = toString(def);
	}
	T read() {
		if (!successfullyWritten) { 
			val = def;
			string s = readAsString();
			fromString(s, val);
		}
		return val;
	}
	operator const T() { return read(); } 
	SPIFFSVariableESP32 & operator=(const T&v) { write(v); return *this; } 
	void write(const T &v) {
		if (val != v) {
			val = v;
			string s = toString(val);
			this->writeAsString(s);
		}
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
	void autoConnect() {
		if (!enabled) 
			return;

		struct {
			const char *name;
			const char *pass;
			const char *mqtt;
		} aps[] = {	{"Ping-582B", "", ""}, 
					{"ChloeNet", "niftyprairie7", ""},
					{"MOF-Guest", "", ""},
					{"ClemmyNet","clementine is a cat", ""},
					{"ChloeNet4", "niftyprairie7", ""},
					{"Station 54", "Local1747", ""},
				 };

		//WiFi.disconnect(true);
		//WiFi.mode(WIFI_STA);
		//WiFi.setSleep(false);
		//delay(100);

		int bestMatch = lastAP;
		if (bestMatch >= 0 && bestMatch < (sizeof(aps)/sizeof(aps[0]))) { 
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
				for (int j = 0; j < sizeof(aps)/sizeof(aps[0]); j++) { 				
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

#ifndef ESP32

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
#endif // ESP32

template<class T>
class ExtrapolationTable {
        bool between(T a, T b, T c) { 
                return (c >= a && c < b) || (c <= a && c > b);
        }
public:
        struct Pair {
             T a,b;        
        } *table;
        ExtrapolationTable(struct Pair *t) : table(t) {}
        T extrapolate(T in, bool reverse = false) {
                for(int index = 1; table[index].a != -1 || table[index].b != -1; index++) {     
                        if (!reverse && between(table[index - 1].a, table[index].a, in))  
                                return table[index - 1].b + (in - table[index - 1].a) 
                                        * (table[index].b - table[index - 1].b) 
                                        / (table[index].a - table[index - 1].a);
                        if (reverse && between(table[index - 1].b, table[index].b, in)) 
                                return table[index - 1].a + (in - table[index - 1].b) 
                                        * (table[index].a - table[index - 1].a) 
                                        / (table[index].b - table[index - 1].b);                
          
                }
                return -1;
        }
        T operator () (T in) { return extrapolate(in); }
};

static const float FEET_PER_METER = 3.28084;
static const float MPS_PER_KNOT = 0.51444;

inline float random01() { 	return rand() / (RAND_MAX + 1.0); }
#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
#ifndef GIT_VERSION
#define GIT_VERSION "unknown-git-version"
#endif


static inline float avgAnalogRead(int pin, int avg = 1024) { 
	float bv = 0;
	for (int i = 0; i < avg; i++) {
		bv += analogRead(pin);
	}
	return bv / avg;
}

static inline void bin2hex(const char *in, int len, char *out, int olen) {
	len = min(len, olen / 2); 
	for (int n = 0; n < len; n++) { 
		sprintf(out + 2 * n, "%02x", in[n]);
	}
	out[2 * len] = '\0';
}

static inline int hex2bin(const char *in, char *out, int inLength) { 
        for (const char *p = in; p < in + inLength ; p += 2) { 
                char b[3];
                b[0] = p[0];
                b[1] = p[1];
                b[2] = 0;
                int c;
                sscanf(b, "%x", &c);
                *(out++) = c;
        }
        return inLength / 2;
}

static inline void dbg(const char *(format), ...) { 
	va_list args;
	va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	//mqtt.publish("debug", buf);
	Serial.println(buf);
}

void webUpgrade(const char *u);

// create a String from a char buf without NULL termination 
static inline String buf2str(const byte *buf, int len) { 
  String s;
  for (int i = 0; i < len; i++) {
	s += (char)buf[i];
  }
  return s;
}

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
	MQTTClient(const char *s, const char *t, std::function<void(String,String)> cb = NULL, bool a = true);
	void publish(const char *suffix, const char *m);
	void publish(const char *suffix, const String &m);
	void pub(const char *m);
	void reconnect();
	void dprintf(const char *format, ...);
	void run();
};

int getLedPin();


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
	QuickRebootCounter quickRebootCounter;
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
	MQTTClient mqtt = MQTTClient("192.168.68.73", basename_strip_ext(__BASE_FILE__).c_str());
	void run() { 
		if (beginRan == false)
			begin();
		quickRebootCounter.run();
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
		LP();
		Serial.printf("\n\n\n%s git:" GIT_VERSION " mac:%s time:%05.3fs built:" __DATE__ " " __TIME__ " \n", 
			basename_strip_ext(__BASE_FILE__).c_str(), getMacAddress().c_str(), millis() / 1000.0);
		getLedPin();
		LP();
		//led.setPercent(30);
		jw.onConnect([this](){
			led.setPattern(500, 2);
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
	void out(const char *format, ...) { 
		va_list args;
		va_start(args, format);
		char buf[1024];
		vsnprintf(buf, sizeof(buf), format, args);
		va_end(args);
		mqtt.pub(buf);
		printf("%s", buf);
		printf("\n");
		jw.udpDebug(buf);
	}
	void log(int ll, const char *format, ...) { 
		va_list args;
		va_start(args, format);
		char buf[1024];
		if (logLevel >= ll) { 
			vsnprintf(buf, sizeof(buf), format, args);
			va_end(args);
			mqtt.pub(buf);
			Serial.println(buf);
			//jw.udpDebug(buf);
		}
	}
	void log(int l, String s) { log(l, s.c_str()); }
	void log(int l, std::string s) { log(l, s.c_str()); }
	void out(String s) { out(s.c_str()); }
	void out(std::string s) { out(s.c_str()); }
};

#define OUT j.out
#define LOG j.log

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

// registers SETTEMP, CURRENTTEMP, HIST commands with CLI.  Returns heat on/off
// value from check() function 
class CliTempControl {
public:
	CliVariable<float> setTemp;
	CliVariable<float> currentTemp;
	CliVariable<float> hist;
	CliVariable<int> heat;
	CliVariable<float> minTemp;
	JStuff *j;

	CliTempControl(JStuff *js, float temp, float h) :
		j(js), setTemp(js->cli, "setTemp", temp),
		currentTemp(js->cli, "currentTemp", temp), 
		hist(js->cli, "hist", h),
		heat(js->cli, "heat", 0),
		minTemp(js->cli, "minTemp", 5) {}

	bool check(float temp) { 
		currentTemp = temp;
		if (temp > setTemp) { 
			heat = 0;
		} else if (temp >= minTemp && temp < setTemp - hist) { 
			heat = 1;
		}
		return heat;
	}	
	void pub() { 
		j->out("setTemp: %6.2f currentTemp: %6.2f heat: %d", (float)setTemp, (float)currentTemp, (int)heat);
	}
};

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


//#endif
#ifdef CSIM
#include "jimlib.cpp"
#endif
#endif //#ifndef INC_JIMLIB_H
