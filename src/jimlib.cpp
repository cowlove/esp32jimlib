#include "jimlib.h" //todo- remove all static defs from jimlib.h

#ifndef CSIM
#include <esp_task_wdt.h>
#include <esp_mac.h>
#include <rom/rtc.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include "Wire.h"
#include <OneWireNg.h>
#include <OneWireNg_CurrentPlatform.h>

#define LittleFS SPIFFS

#else
#include "ESP32sim_ubuntu.h" // sigh remove all static defs here too
#endif


void ledcInit(int pin, int freq, int res, int channel) {
#if ESP_ARDUINO_VERSION_MAJOR == 3 
	ledcAttachChannel(pin, freq, res, channel);
#else
	ledcSetup(channel, freq, res);
	ledcAttachPin(pin, channel);
#endif
}

void wdtInit(int sec) {
#if ESP_ARDUINO_VERSION_MAJOR == 3
	esp_task_wdt_config_t c; 
	c.timeout_ms = (sec)*1000; 
	c.idle_core_mask = 0x1; 
	c.trigger_panic = true; 
	esp_task_wdt_deinit(); 
	esp_task_wdt_init(&c);  // include jimlib.h last or this will cause compile errors in other headers
	esp_task_wdt_add(NULL);
#else	
	esp_task_wdt_init(sec, true);
	esp_task_wdt_add(NULL);
#endif
}

void wdtReset() { 
	esp_task_wdt_reset();
}

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
	for (uint8_t i = 8; i < 120; i++)
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

int LineBuffer::add(char c, std::function<void(const char *)> f/* = NULL*/) {
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
void LineBuffer::add(const char *b, int n, std::function<void(const char *)> f) { 
	for (int i = 0; i < n; i++) {
		add(b[i], f); 
	}
}
void LineBuffer::add(const uint8_t *b, int n, std::function<void(const char *)> f) { 
	add((const char *)b, n, f); 
} 

#ifdef ESP32 // TODO we could do this on the 8266
void webUpgrade(const char *u) {
	//WiFiClientSecure wc;
	//wc.setInsecure();
	HTTPClient client; 

	int offset = 0;
	int len = 1024 * 16;
	int errors = 0;
	int fwLen = 0;
 
	;
	if (Update.begin(UPDATE_SIZE_UNKNOWN) == false) { 
		dbg("Update.begin() failed with %s", Update.errorString());
	}
	Serial.println("Updating firmware...");

	while(true) { 
		esp_task_wdt_reset();
		String url = String(u) + Sfmt("?len=%d&offset=%d", len, offset);
		dbg("offset %d, len %d, url %s", offset, len, url.c_str());
		//client.begin(wc, url);
		client.begin(url);
		int resp = client.GET();
		//dbg("HTTPClient.get() returned %d", resp);
		if(resp != 200) {
			dbg("Get failed\n");
			Serial.print(client.getString());
			delay(5000);
			if (++errors > 10) { 
				return;
			}
			continue;
		}
		int	totalLength = client.getSize();
		int len = totalLength;
		uint8_t bbuf[128], tbuf[256];
	
		//Serial.printf("FW Size: %u\n",totalLength);
		if (totalLength == 0) { 
			Serial.printf("\nUpdate Success, Total Size: %u\nRebooting...\n", fwLen);
			if (Update.end(true) == false) { 
				dbg("Update.end() failed with %s", Update.errorString());
			};
			ESP.restart();
			return;				
		}
				
		WiFiClient * stream = client.getStreamPtr();
		while(client.connected() && len > 0) {
			int avail = stream->available();
			avail = avail & 0xfffffe; // don't split 2-byte hex across buffers
			//dbg("Avail %d %d", stream->available(), avail);
			if(avail) {
				int c = stream->readBytes(tbuf, ((avail > sizeof(tbuf)) ? sizeof(tbuf) : avail));
				if (c > 0) {
					hex2bin((const char *)tbuf, (char *)bbuf, c);
					//if (c % 2 == 1) 
					//	dbg("Update with %d, avail %d", c, avail);
					Update.write(bbuf, c / 2);
					fwLen += c / 2;
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

//int SpiffsInit = 0;

#ifdef GIT_VERSION
char _GIT_VERSION[] = GIT_VERSION;
#else
char _GIT_VERSION[] = "undefined";
#endif

MQTTClient::MQTTClient(const char *s, const char *t, 
	std::function<void(String,String)> cb/* = NULL*/, bool a/*= true*/) : 
active(a), server(s), topicPrefix(t), userCallback(cb) {
	client = new PubSubClient(espClient);
}
void MQTTClient::callBack(char *topic, byte *p, unsigned int l) {
	if (userCallback != NULL) { 
		userCallback(String(topic), buf2str(p, l));
	}		
}

void MQTTClient::publish(const char *suffix, const char *m) { 
	if (active) { 
		String t = topicPrefix + "/" + suffix;
		client->publish(t.c_str(), m);
	}
}
void MQTTClient::publish(const char *suffix, const String &m) {
	 publish(suffix, m.c_str()); 
}
void MQTTClient::pub(const char *m) { publish("out", m); } 
void MQTTClient::reconnect() {
// Loop until we're reconnected
	if (active == false || WiFi.status() != WL_CONNECTED || client->connected()) 
		return;
	client->setServer(server.c_str(), 1883);
	Serial.println("MQTT connecting...");
	if (client->connect((topicPrefix + getMacAddress()).c_str())) {
		client->subscribe((topicPrefix + "/in").c_str());
		client->setCallback([this](char* topic, byte* p, unsigned int l) {
			this->callBack(topic, p, l);
		});
		std::string s = strfmt("MQTT connected uptime %.1f sec %s %s %s " GIT_VERSION " built " __DATE__ " " __TIME__ , 
			millis() / 1000.0, getMacAddress().c_str(),  
			WiFi.localIP().toString().c_str(), basename_strip_ext(__BASE_FILE__).c_str());
		publish("sys", s.c_str());
		Serial.println(s.c_str());
	} else {
		Serial.print("failed, rc=");
		Serial.print(client->state());
	}
}
void MQTTClient::dprintf(const char *format, ...) { 
	va_list args;
	va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	client->publish((topicPrefix + "/debug").c_str(), buf);
}
void MQTTClient::run() { 
	if (active) { 
		client->loop();
		reconnect();
	}
}


const String &getMacAddress() {
	static uint8_t baseMac[6] = {0xff};
#ifdef ESP32
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
#endif
	static char baseMacChr[18] = {0};
	static String mac;
	sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	mac = baseMacChr;
	return mac;
}

SPIFFSVariableESP32Base::SPIFFSVariableESP32Base() { 
	LittleFS.begin();
}

void SPIFFSVariableESP32Base::begin() { 
	LittleFS.begin();
	initialized = true;
}
bool SPIFFSVariableESP32Base::initialized = false;

void SPIFFSVariableESP32Base::writeAsString(const string &s) { 
	fs::File file = LittleFS.open(filename.c_str(), "w");
	if (file) { 
		int r = file.write((const uint8_t *)s.c_str(), s.length());
		file.flush();
		file.close();
		//printf("SPIFFSVariableESP32 write returned %d file %s %s\n", r, filename.c_str(), s.c_str());
	} else if(initialized) { 
		printf("SPIFFSVariableESP32 error writing file %s, formatting SPIFFS\n", filename.c_str());
		LittleFS.format();
		LittleFS.begin();
		file = LittleFS.open(filename.c_str(), "w");
		if (file) { 
			file.write((const uint8_t *)s.c_str(), s.length());
			file.flush();
			file.close();	
		}
	}
}

string SPIFFSVariableESP32Base::readAsString() { 
	//printf("reading file %s\n", filename.c_str());
	string rval;
	fs::File file = LittleFS.open(filename.c_str(), "r");
	size_t bytes_read = 0;
	if (file) { 
		while(true) { 
			uint8_t buf[64];  
			int r = file.read(buf, sizeof(buf) - 1);
			//printf("read() returned %d from %s\n", r, filename.c_str());
			if (r <= 0)
				break;
			buf[r] = 0;
			rval += string((char *)buf); 
			//printf("SPIFFSVariableESP32 read %d bytes from file %s %s\n", r, filename.c_str(), buf);
		}
	} else { 
		printf("Error opening file %s\n", filename.c_str());
	}
	if (rval.length() == 0) {
		writeAsString(defaultStringValue);
		rval = defaultStringValue;
	}
	return rval;
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

static inline void swapEndian(void *p1, void *p2, int s) { 
	for(int n = 0; n < s; n++) 
		((char *)p1)[n] = ((char *)p2)[s - n - 1];
}

std::vector<DsTempData> readTemps(OneWireNg *ow) { 
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

int getLedPin() { 
	const String mac = getMacAddress(); 
	if (mac == "349454C12BB0") return 5; // Lilygo LiPo board
	if (mac == "9C9C1FC9BE94") return 2;
	if (mac == "9C9C1FCB0920") return 2;
	if (mac == "2462ABDDCB34") return 19; // TTGO 1.8 TFT board 
	//Serial.printf("MAC %s not found in getLedPin, defaulting to pin 2\n", mac.c_str());
	return 2;
}


int getResetReason(int cpu) { return rtc_get_reset_reason(cpu); }

const char *reset_reason_string(int reason) {
  switch ( reason)
  {
    case 1 : return ("POWERON_RESET");break;          /**<1, Vbat power on reset*/
    case 3 : return("SW_RESET");break;               /**<3, Software reset digital core*/
    case 4 : return ("OWDT_RESET");break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : return ("DEEPSLEEP_RESET");break;        /**<5, Deep Sleep reset digital core*/
    case 6 : return ("SDIO_RESET");break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : return ("TG0WDT_SYS_RESET");break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : return ("TG1WDT_SYS_RESET");break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : return ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/
    case 10 : return ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : return ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : return ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : return ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : return ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : return ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : return ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
  }
  printf("unknown reset reason %d\n", reason);
  return ("UNKNOWN_RESET_REASON");
}
