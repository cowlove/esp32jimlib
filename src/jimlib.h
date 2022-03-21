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
#ifdef ESP32
#include <esp_task_wdt.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <Update.h>			
#include <WebServer.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <mySD.h> // Add "EXCLUDE_DIRS=esp32-micro-sdcard" to Makefile if this breaks ESP8266 builds
#endif //ESP32
#endif // !UBUNTU

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
	int add(char c) {
		int r = 0;
		if (c != '\r' && c != '\n')
			line[len++] = c; 
		if (len >= sizeof(line) - 1 || c == '\n') {
			r = len;
			line[len] = 0;
			len = 0;
		}
		return r;
	}
	void add(const char *b, int n, std::function<void(const char *)> f) { 
		for (int i = 0; i < n; i++) {
			if (add(b[i])) 
				f(line);
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
public:
	EggTimer(float ms) : interval(ms * 1000), last(0) { reset(); }
	bool tick() { 
		uint64_t now = micros();
		if (now - last >= interval) { 
			last += interval;
			// reset last to now if more than one full interval behind 
			if (now - last >= interval) 
				last = now;
			return true;
		} 
		return false;
	}
	void reset() { 
		last = micros();
	}
	void alarmNow() { 
		last = 0;
	}
};


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
		value += delta;
		if (value < limMin) value = wrap ? limMax : limMin;
		if (value > limMax) value = wrap ? limMin : limMax;
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
        if (!(ec == OneWireNg::EC_MORE || ec == OneWireNg::EC_NO_DEVS))
            break;

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


#ifdef ESP32
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
  "processData:false,"
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



class JimWiFi { 
	EggTimer report = EggTimer(1000);
	bool firstRun = true, firstConnect = true;
	std::function<void(void)> connectFunc = NULL;
	std::function<void(void)> otaFunc = NULL;
	bool updateInProgress = false;
public:
	WiFiUDP udp;
	WiFiMulti wifi;
	void onConnect(std::function<void(void)> oc) { 
		connectFunc = oc;
	}
	void onOTA(std::function<void(void)> oo) { 
		otaFunc = oo;
	}
	void run() { 
		if (firstRun) {
			WiFi.disconnect(true);
			WiFi.mode(WIFI_STA);
			WiFi.setSleep(false);

			wifi.addAP("ChloeNet", "niftyprairie7");
			//wifi.addAP("TUK-FIRE", "FD priv n3t 20 q4");
			firstRun = false;
		}
		wifi.run();
		if (WiFi.status() == WL_CONNECTED) { 
			if (firstConnect ==  true) { 
				firstConnect = false;
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
				WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector   

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
			server.handleClient();
			if (report.tick()) {
				udpDebug(""); 
			}
		}
	}
	bool connected() { return WiFi.status() == WL_CONNECTED;  }  
	void udpDebug(const char *s) { 
		if (!connected())
			return;
		udp.beginPacket("255.255.255.255", 9000);
		char b[128];
		snprintf(b, sizeof(b), "%d %s    " __BASE_FILE__ "   " __DATE__ "   " __TIME__ "   0x%08x: ", 
			(int)(millis() / 1000), WiFi.localIP().toString().c_str(), /*(int)ESP.getEfuseMac()*/0);
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

#ifdef ESP32

template<class T> 
class SPIFFSVariable { 
	String filename;
	const T def;
public:
	SPIFFSVariable(const char *f, const T &d) : filename(f), def(d) {}
	operator const T() {
		T val = def;
		fs::File file = SPIFFS.open(filename.c_str(), "r");
		if (file) { 
			uint8_t buf[64];
			int b = file.read(buf, sizeof(buf) - 1);
			Serial.printf("read %d bytes\n");
			file.close();
			if (b > 0) {
				buf[b] = 0; 
				sscanf((char *)buf, "%d", &val);
			}
		}
		return val;
	} 
	SPIFFSVariable & operator=(const T&v) { 
		fs::File file = SPIFFS.open(filename.c_str(), "w");
		if (file) { 
			file.printf("%d\n", v);
			file.close();
		} else { 
			Serial.printf("error writing file %s\n", filename.c_str());
			SPIFFS.format();
		}
		return *this;
	}
};
#endif

class ShortBootDebugMode {
	SPIFFSVariable<int> shortBootCount = SPIFFSVariable<int>("/shortBootCount", 1);
	bool initialized = false;
	bool cleared = false;
	int sbCount; 
	int threshold;
  public:
	ShortBootDebugMode(int t) : threshold(t) {}
	void begin() {
		SPIFFS.begin();
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
char _GIT_VERSION[] = "GIT_VERSION " GIT_VERSION;

#endif

//#endif
#endif //#ifndef INC_JIMLIB_H
