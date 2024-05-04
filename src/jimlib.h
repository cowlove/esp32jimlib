#ifndef INC_JIMLIB_H
#define INC_JIMLIB_H
#include <functional>
#include <string>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <fcntl.h>
#ifndef UBUNTU
#include "DNSServer.h"
#include <HardwareSerial.h>
#include <SPI.h>
#define FS_NO_GLOBALS
#include <FS.h>
#include "ArduinoOTA.h"
#include "WiFiUdp.h"
#include "Wire.h"
#include <OneWireNg.h>
#include <OneWireNg_CurrentPlatform.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <utility>
#include <regex>

#ifdef ESP32
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <Update.h>			
#include <HTTPClient.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <Update.h>			
#include <WebServer.h>
#include <mySD.h> // Add "EXCLUDE_DIRS=esp32-micro-sdcard" to Makefile if this breaks ESP8266 builds
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#else // ESP32
static inline void esp_task_wdt_reset() {}
static inline void esp_task_wdt_init(int, int) {}
static inline void esp_task_wdt_add(void *) {}
static inline void ledcSetup(int, int, int) {}
static inline void ledcAttachPin(int, int) {}
static inline void ledcWrite(int, int) {}
#endif //ESP32
#else // !UBUNTU
#include "ESP32sim_ubuntu.h"
#endif

#include <stdarg.h>
#define DEG2RAD(x) ((x)*M_PI/180)
#define RAD2DEG(x) ((x)*180/M_PI)

std::string strfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
	return std::string(buf);
}

String Sfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
	return String(buf);
}

std::string sfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
	return std::string(buf);
}

int scanI2c() { 
	int count = 0;
	for (byte i = 8; i < 120; i++)
	{
			Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
			if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
			{
					Serial.print ("Found address: ");
					Serial.print (i, DEC);
					Serial.print (" (0x");
					Serial.print (i, HEX);     // PCF8574 7 bit address
					Serial.println (")");
					count++;
			}
	}
	Serial.print ("Found ");      
	Serial.print (count, DEC);        // numbers of devices
	Serial.println (" device(s).");

	return count;
}

void printPins() { 
        for (int n = 0; n <= 1; n++) {
                pinMode(n, INPUT_PULLUP); 
                Serial.printf("%02d:%d ", n, digitalRead(n));
        }
        for (int n = 2; n <= 5; n++) {
                pinMode(n, INPUT_PULLUP); 
                Serial.printf("%02d:%d ", n, digitalRead(n));
        }
        for (int n = 12; n <= 39; n++) { 
                pinMode(n, INPUT_PULLUP); 
                Serial.printf("%02d:%d ", n, digitalRead(n));
        }
        Serial.print("\n");
#ifdef ESP32
		esp_task_wdt_reset();
#endif
}

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
	int add(char c, std::function<void(const char *)> f = NULL) {
		int r = 0;
		if (c != '\r' && c != '\n')
			line[len++] = c; 
		if (len >= sizeof(line) - 1 || c == '\n') {
			r = len;
			line[len] = 0;
			len = 0;
			if (f != NULL) { 
				f(line);
			}
		}
		return r;
	}
	void add(const char *b, int n, std::function<void(const char *)> f) { 
		for (int i = 0; i < n; i++) {
			add(b[i], f); 
		}
	}
	void add(const uint8_t *b, int n, std::function<void(const char *)> f) { add((const char *)b, n, f); } 
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

#ifndef UBUNTU
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
void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
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

std::string nmeaChecksum(const std::string &s) { 
	char check = 0;
	for (const char &c : s)  
		check ^= c;
	char buf[8];
	snprintf(buf, sizeof(buf), "*%02X\n", (int)check);
	return std::string("$") + s + std::string(buf);	
}

//
// DS18 one-wire temperature sensor library


/* DS therms commands */
#define CMD_CONVERT_T           0x44
#define CMD_COPY_SCRATCHPAD     0x48
#define CMD_WRITE_SCRATCHPAD    0x4E
#define CMD_RECALL_EEPROM       0xB8
#define CMD_READ_POW_SUPPLY     0xB4
#define CMD_READ_SCRATCHPAD     0xBE

/* supported DS therms families */
#define DS18S20     0x10
#define DS1822      0x22
#define DS18B20     0x28
#define DS1825      0x3B
#define DS28EA00    0x42

inline void swapEndian(void *p1, void *p2, int s) { 
	for(int n = 0; n < s; n++) 
		((char *)p1)[n] = ((char *)p2)[s - n - 1];
}

struct DsTempData { 
	uint64_t id;
	uint64_t time;
	float degC;
};
inline std::vector<DsTempData> readTemps(OneWireNg *ow) { 
	std::vector<DsTempData> rval;
    OneWireNg::Id id;
    OneWireNg::ErrorCode ec;
    ow->searchReset();

    do
    {
        ec = ow->search(id);
		//Serial.printf("ec: %d\n", ec);
        //if (!(ec == OneWireNg::EC_MORE || ec == OneWireNg::EC_NO_DEVS))
        //    break;

        /* start temperature conversion */
        ow->addressSingle(id);
        ow->writeByte(CMD_CONVERT_T);

        delay(750);

        uint8_t touchScrpd[] = {
            CMD_READ_SCRATCHPAD,
            /* the read scratchpad will be placed here (9 bytes) */
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
        };

        ow->addressSingle(id);
        ow->touchBytes(touchScrpd, sizeof(touchScrpd));
        uint8_t *scrpd = &touchScrpd[1];  /* scratchpad data */

        if (OneWireNg::crc8(scrpd, 8) != scrpd[8]) {
            //Serial.println("  Invalid CRC!");
            continue;
        }

        long temp = ((long)(int8_t)scrpd[1] << 8) | scrpd[0];

        if (id[0] != DS18S20) {
            unsigned res = (scrpd[4] >> 5) & 3;
            temp = (temp >> (3-res)) << (3-res);  /* zeroed undefined bits */
            temp = (temp*1000)/16;
        } else
        if (scrpd[7]) {
            temp = 1000*(temp >> 1) - 250;
            temp += 1000*(scrpd[7] - scrpd[6]) / scrpd[7];
        } else {
            /* shall never happen */
            temp = (temp*1000)/2;
			continue;
        }
		
		uint64_t lid;
		swapEndian(&lid, &id, sizeof(lid));
		float ftemp = temp / 1000.0;
		DsTempData dsData;
		dsData.degC = temp / 1000.0;
		dsData.id = lid;
		dsData.time = millis();
		
		rval.push_back(dsData);
    } while (ec == OneWireNg::EC_MORE);
    return rval;
}

String basename_strip_ext(const char *fn) {
	String rval(fn); 
	char *p = strrchr((char *)fn, '/');
	if (p != NULL) 
		rval = p + 1;
	if (rval.indexOf('.')) {
		rval = rval.substring(0, rval.indexOf('.'));
	}
	return rval;
}

String getMacAddress() {
	uint8_t baseMac[6] = {0xff};
#ifdef ESP32
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
#endif
	char baseMacChr[18] = {0};
	sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	return String(baseMacChr);
}


#if 0 
const char* host = "esp32";
const char* ssid = "xxx";
const char* password = "xxxx";

WebServer server(80);

/*
 * Login page
 */

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "(process)Data:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
#endif

int spiffsInit = 0;

template<class T> 
class SPIFFSVariableESP32 { 
	String filename;
	const T def;
public:
	SPIFFSVariableESP32(const char *f, const T &d) : filename(f), def(d) {}
	operator const T() {
		T val = def;
		if (!spiffsInit) { 
			SPIFFS.begin();
		
			spiffsInit = 1;
		}
		fs::File file = SPIFFS.open(filename.c_str(), "r");
		if (file) { 
			uint8_t buf[64];
			int b = file.read(buf, sizeof(buf) - 1);
			//Serial.printf("read %d bytes from %s\n", b, filename.c_str());
			file.close();
			if (b > 0) {
				buf[b] = 0; 
				sscanf((char *)buf, "%d", &val);
			}
		}
		return val;
	} 
	SPIFFSVariableESP32 & operator=(const T&v) { 
		if (!spiffsInit) { 
			SPIFFS.begin();
			spiffsInit = 1;
		}	
		fs::File file = SPIFFS.open(filename.c_str(), "w");
		if (file) { 
			file.printf("%d\n", v);
			file.close();
			//Serial.printf("Wrote file %s\n", filename.c_str());
		} else { 
			//Serial.printf("error writing file %s\n", filename.c_str());
			SPIFFS.format();
		}
		return *this;
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
	void autoConnect() {
		if (!enabled) 
			return;

			struct {
				const char *name;
				const char *pass;
			} aps[] = {	{"Ping-582B", ""}, 
						{"ChloeNet", "niftyprairie7"},
						//{"Flora2", "Maynards."},
						//{"MOF-Guest", ""},
						//{"XXX Bear Air Sport Aviation", "niftyprairie7"}, 
						{"trex","Maeby1989!"},
						{"ChloeNet3", "niftyprairie7"},
						{"Tip of the Spear", "51a52b5354"},  
						{"Team America", "51a52b5354"},  
						{"Station 54", "Local1747"},
						{"TUK-FIRE", "FD priv n3t 20 q4"} };

		WiFi.disconnect(true);
		WiFi.mode(WIFI_STA);
		WiFi.setSleep(false);

		int bestMatch = lastAP;
		if (bestMatch >= 0 && bestMatch < (sizeof(aps)/sizeof(aps[0]))) { 
			Serial.printf("Trying cached WiFi AP '%s'...\n", aps[bestMatch].name);
			WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
			for(int d = 0; d < 80; d++) { 
				if (WiFi.status() == WL_CONNECTED) 
					return;
				delay(100);
			}
		}
		bestMatch = -1;

		Serial.println("Scanning...");
		WiFi.disconnect(true);
		WiFi.mode(WIFI_STA);
		WiFi.setSleep(false);
		int n = WiFi.scanNetworks();
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
		Serial.printf("Using WiFi AP '%s'...\n", aps[bestMatch].name);
		WiFi.disconnect(true);
		WiFi.mode(WIFI_STA);
		WiFi.setSleep(false);
		delay(100);
		WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
		lastAP = bestMatch;
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
		if (connCheck.tick() && WiFi.status() != WL_CONNECTED) {
			WiFi.disconnect();
			WiFi.reconnect();
			firstRun = false;
		}
		if (WiFi.status() == WL_CONNECTED) { 
			if (firstConnect ==  true) { 
				firstConnect = false;
#if  0
				server.on("/", HTTP_GET, []() {
				server.sendHeader("Connection", "close");
				server.send(200, "text/html", loginIndex);
				});
				server.on("/serverIndex", HTTP_GET, []() {
				server.sendHeader("Connection", "close");
				server.send(200, "text/html", serverIndex);
				});
				/*handling uploading firmware file */
				server.on("/update", HTTP_POST, []() {
				server.sendHeader("Connection", "close");
				server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
				ESP.restart();
				}, [this]() {
				HTTPUpload& upload = server.upload();
				esp_task_wdt_reset();

				if (upload.status == UPLOAD_FILE_START) {
					if (this->otaFunc != NULL) { 
						this->otaFunc();
					}
					Serial.printf("Update: %s\n", upload.filename.c_str());
					if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
					Update.printError(Serial);
					}
				} else if (upload.status == UPLOAD_FILE_WRITE) {
					//Serial.printf("Update: write %d bytes\n", upload.currentSize);
					esp_task_wdt_reset();	
					/* flashing firmware to ESP*/
					if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
					Update.printError(Serial);
					}
					delay(10);
				} else if (upload.status == UPLOAD_FILE_END) {
					if (Update.end(true)) { //true to set the size to the current progress
					Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
					} else {
					Update.printError(Serial);
					}
				}
				});
				server.begin();
#endif

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
					WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector   
					#endif

					// NOTE: if updating FS this would be the place to unmount FS using FS.end()
					Serial.println("Start updating " + type);
					});
					ArduinoOTA.onEnd([]() {
					Serial.println("\nEnd");
				});
				ArduinoOTA.onProgress([&](unsigned int progress, unsigned int total) {
						//Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
						esp_task_wdt_reset();
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

#ifdef UBUNTU
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

static const float FEET_PER_METER = 3.3208;
static const float MPS_PER_KNOT = 0.51444;

float random01() { 	return rand() / (RAND_MAX + 1.0); }
#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))

#ifdef GIT_VERSION
char _GIT_VERSION[] = GIT_VERSION;
#else
char _GIT_VERSION[] = "undefined";
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

void dbg(const char *(format), ...) { 
	va_list args;
	va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	//mqtt.publish("debug", buf);
	Serial.println(buf);
}

#ifdef ESP32 // TODO we could do this on the 8266
static void webUpgrade(const char *u) {
	WiFiClientSecure wc;
	wc.setInsecure();
	HTTPClient client; 

	int offset = 0;
	int len = 1024 * 16;
	int errors = 0;
 
	Update.begin(UPDATE_SIZE_UNKNOWN);
	Serial.println("Updating firmware...");

	while(true) { 
		esp_task_wdt_reset();
		String url = String(u) + Sfmt("?len=%d&offset=%d", len, offset);
		dbg("offset %d, len %d, url %s", offset, len, url.c_str());
		client.begin(wc, url);
		int resp = client.GET();
		//dbg("HTTPClient.get() returned %d\n", resp);
		if(resp != 200) {
			dbg("Get failed\n");
			Serial.print(client.getString());
			delay(5000);
			if (++errors > 10) { 
				return;
			}
			continue;
		}
		int currentLength = 0;
		int	totalLength = client.getSize();
		int len = totalLength;
		uint8_t bbuf[128], tbuf[256];
	
		//Serial.printf("FW Size: %u\n",totalLength);
		if (totalLength == 0) { 
			Serial.printf("\nUpdate Success, Total Size: %u\nRebooting...\n", currentLength);
			Update.end(true);
			ESP.restart();
			return;				
		}
				
		WiFiClient * stream = client.getStreamPtr();
		while(client.connected() && len > 0) {
			size_t avail = stream->available();
			if(avail) {
				int c = stream->readBytes(tbuf, ((avail > sizeof(tbuf)) ? sizeof(tbuf) : avail));
				if (c > 0) {
					hex2bin((const char *)tbuf, (char *)bbuf, c);
					//dbg("Update with %d", c / 2);
					Update.write(bbuf, c / 2);
					if(len > 0) {
						len -= c;
					}
				}
			}
			delay(1);
		}	
		client.end();
		offset += totalLength / 2;
	}
}
#endif

// create a String from a char buf without NULL termination 
String buf2str(const byte *buf, int len) { 
  String s;
  for (int i = 0; i < len; i++) {
	s += (char)buf[i];
  }
  return s;
}

// connects to an mqtt server, publish stuff on <name>, subscribe to <name>/in

class MQTTClient { 
	WiFiClient espClient;
	void callBack(char *topic, byte *p, unsigned int l) {
		if (userCallback != NULL) { 
			userCallback(String(topic), buf2str(p, l));
		}		
	}
	std::function<void(String,String)> userCallback = NULL;
public:
        String topicPrefix, server;
	bool active;
	PubSubClient client;
	void setCallback(std::function<void(String,String)> cb) { userCallback = cb; }
	MQTTClient(const char *s, const char *t, std::function<void(String,String)> cb = NULL, bool a = true) : 
		active(a), server(s), topicPrefix(t), client(espClient), userCallback(cb) {
	}
	void publish(const char *suffix, const char *m) { 
		String t = topicPrefix + "/" + suffix;
		client.publish(t.c_str(), m);
	}
	void publish(const char *suffix, const String &m) {
		 publish(suffix, m.c_str()); 
	}
	void pub(const char *m) { publish("out", m); } 
	void reconnect() {
	// Loop until we're reconnected
		if (active == false || WiFi.status() != WL_CONNECTED || client.connected()) 
			return;
		client.setServer(server.c_str(), 1883);
		Serial.println("MQTT connecting...");
		if (client.connect((topicPrefix + getMacAddress()).c_str())) {
			client.subscribe((topicPrefix + "/in").c_str());
			client.setCallback([this](char* topic, byte* p, unsigned int l) {
				this->callBack(topic, p, l);
			});
			std::string s = strfmt("MQTT connected uptime %.1f sec", millis() / 1000.0);
			publish("sys", s.c_str());
			Serial.println(s.c_str());
		} else {
			Serial.print("failed, rc=");
			Serial.print(client.state());
		}
	}
	void dprintf(const char *format, ...) { 
		va_list args;
		va_start(args, format);
        char buf[256];
        vsnprintf(buf, sizeof(buf), format, args);
	    va_end(args);
		client.publish((topicPrefix + "/debug").c_str(), buf);
	}
	void run() { 
		if (active) { 
			client.loop();
			reconnect();
		}
	}
};

#ifndef ESP32
#define PROGMEM
#endif

int getLedPin() { 
	const String mac = getMacAddress(); 
	if (mac == PROGMEM "349454C12BB0") return 5; // Lilygo LiPo board
	if (mac == PROGMEM "9C9C1FC9BE94") return 2;
	if (mac == PROGMEM "9C9C1FCB0920") return 2;
	if (mac == PROGMEM "2462ABDDCB34") return 19; // TTGO 1.8 TFT board 
	Serial.printf("MAC %s not found in getLedPin, defaulting to pin 2\n", mac.c_str());
	return 2;
}


class PwmChannel {
	int pin; 
	int channel;
	int pwm = -1;
public:		
	int gradual;
	PwmChannel(int p, int hz = 50, int c = 0, int g = 0) : pin(p), channel(c), gradual(g) {
		ledcSetup(channel, hz, 16);
		ledcAttachPin(pin, channel);
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

#ifndef ESP32
#define PROGMEM
#endif

#if 1 // TODO - this could move back to supporting ESP8266
using namespace std;

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

template<> const char *CommandLineInterfaceESP32::formatOf<float>() { return "%f"; }
template<> const char *CommandLineInterfaceESP32::formatOf<int>() { return "%i"; }

template<>
void CommandLineInterfaceESP32::hookRaw<string>(const char *pat, string *v) {
	on(pat, [v](const char *, smatch m) { 
		if (m.size() > 1 && m.str(1).length() > 0)
			*v = m.str(1).c_str();
		return string(*v);
	});
}
template<>
void CommandLineInterfaceESP32::hookRaw<String>(const char *pat, String *v) {
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

#define CLI_VARIABLE_INT(name,val) CliVariable<int> name = CliVariable<int>(j.cli, #name, val)
#define CLI_VARIABLE_FLOAT(name,val) CliVariable<float> name = CliVariable<float>(j.cli, #name, val)
#define CLI_VARIABLE_STRING(name,val) CliVariable<String> name = CliVariable<String>(j.cli, #name, val)

class JStuff {		
public:
	bool parseSerial;
	std::function<void()> onConn = NULL;
	LineBuffer lb;
	bool debug = false;
	CommandLineInterface cli;
	CliVariable<int> logLevel;
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

	bool cliEcho = true;
	JimWiFi jw;
	MQTTClient mqtt = MQTTClient("192.168.5.1", basename_strip_ext(__BASE_FILE__).c_str());
	void run() { 
		lastRun = thisRun;
		thisRun = micros();
		forceTicksNow = false;

		esp_task_wdt_reset();
		jw.run(); 
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
		esp_task_wdt_init(60, true);
		esp_task_wdt_add(NULL);

		Serial.begin(921600, SERIAL_8N1);
		Serial.println(__BASE_FILE__ " " GIT_VERSION " " __DATE__ " " __TIME__);
		getLedPin();

		led.setPercent(30);
		jw.onConnect([this](){
			led.setPattern(500, 2);
			if (WiFi.SSID() == "ChloeNet" || WiFi.SSID() == "FakeWiFi") {
				jw.debug = mqtt.active = true;  
				out("JStuff: forcing debug and mqtt.active due to WiFi network");
 			}
			Serial.printf("Connected to AP '%s' in %dms, IP=%s\n",
				WiFi.SSID().c_str(), millis(), WiFi.localIP().toString().c_str());
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
		char buf[256];
		vsnprintf(buf, sizeof(buf), format, args);
		va_end(args);
		mqtt.pub(buf);
		Serial.println(buf);
		//jw.udpDebug(buf);
	}
	void log(int ll, const char *format, ...) { 
		va_list args;
		va_start(args, format);
		char buf[256];
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

void digitalToggle(int pin) { pinMode(pin, OUTPUT); digitalWrite(pin, !digitalRead(pin)); }

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



//#endif
#endif //#ifndef INC_JIMLIB_H
