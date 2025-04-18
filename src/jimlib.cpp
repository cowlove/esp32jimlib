#include "jimlib.h" //todo- remove all static defs from jimlib.h
#include "serialLog.h"
#include <sstream>

#ifndef CSIM
#include <esp_task_wdt.h>
#include <esp_mac.h>
#include <rom/rtc.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "Wire.h"
#include <OneWireNg.h>
#include <OneWireNg_CurrentPlatform.h>
#include "driver/ledc.h"
//#include "rom/uart.h"
#include <HTTPClient.h>
#include <esp_sleep.h>



//#include <SPIFFS.h>
//#define LittleFS SPIFFS
#include <LittleFS.h>
#endif // #ifndef CSIM

void wdtReset();
void printPins();
int scanI2c();
std::string sfmt(const char *format, ...);
std::string vsfmt(const char *format, va_list args);
String Sfmt(const char *format, ...);
void wdtInit(int sec);
void ledcInit(int pin, int freq, int res, int channel);

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

void wdtAdd() { 
	esp_task_wdt_add(NULL);
}
void wdtReset() { 
	esp_task_wdt_reset();
}

String Sfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	char buf[256];
	vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
	return String(buf);
}

std::string vsfmt(const char *format, va_list args) {
	va_list args2;
	va_copy(args2, args);
	char buf[128]; // don't understand why stack variable+copy is faster
	string rval;

	int n = vsnprintf(buf, sizeof(buf), format, args);
	if (n > sizeof(buf) - 1) {
		rval.resize(n + 2, ' ');
		vsnprintf((char *)rval.data(), rval.size(), format, args2);
		//printf("n %d size %d strlen %d\n", n, (int)rval.size(), (int)strlen(rval.c_str()));
		rval.resize(n);
	} else { 
		rval = buf;
	}
	va_end(args2);
	return rval;
}

std::string sfmt(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	string rval = vsfmt(format, args);
	va_end(args);
	return rval;
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
	wdtReset();
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
		wdtReset();
		String url = String(u) + Sfmt("?len=%d&offset=%d", len, offset);
		OUT("offset %d, len %d, url %s", offset, len, url.c_str());
		//client.begin(wc, url);
		client.begin(url);
		int resp = client.GET();
		OUT("HTTPClient.get() returned %d", resp);
		if(resp != 200) {
			wdtReset();
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

MQTTClient::MQTTClient() {
	client = new PubSubClient(espClient);
}
void MQTTClient::begin(	const char *s, const char *t, 
	std::function<void(String,String)> cb/* = NULL*/, bool a/*= true*/) {
	active = a;
	server = s; 
	topicPrefix = t;
	userCallback = cb;
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
	Serial.printf("MQTT connecting to server '%s'...\n", server.c_str());
	if (client->connect((topicPrefix + getMacAddress()).c_str())) {
		client->subscribe((topicPrefix + "/in").c_str());
		client->setCallback([this](char* topic, byte* p, unsigned int l) {
			this->callBack(topic, p, l);
		});
		std::string s = strfmt("MQTT connected uptime %.1f sec %s %s %s " GIT_VERSION, 
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
	static char baseMacChr[32] = {0};
	static String mac;
	snprintf(baseMacChr, sizeof(baseMacChr), "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	mac = baseMacChr;
	return mac;
}

SPIFFSVariableESP32Base::SPIFFSVariableESP32Base() {
	if (filename.length() > 31) { 
		printf("SPIFFSVariable WARNING filename '%s' length %d > 31\n", filename.c_str(), (int)filename.length());
	} 
	//LittleFS.begin();
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
		if (r != s.length()) { 
			printf("SPIFFSVariableESP32 write fail: returned %d file %s '%s'\n", r, filename.c_str(), s.c_str());
		}
		file.flush();
		file.close();
		successfullyWritten = true;
		if (debug)
			printf("SPIFFSVariableESP32 write returned %d file %s %s\n", r, filename.c_str(), s.c_str());
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
			uint8_t buf[96];  
			int r = file.read(buf, sizeof(buf) - 1);
			//printf("read() returned %d from %s\n", r, filename.c_str());
			if (r <= 0)
				break;
			buf[r] = 0;
			rval += string((char *)buf); 
			if (debug)
				printf("SPIFFSVariableESP32 read %d bytes from file %s '%s'\n", r, filename.c_str(), buf);
		}
	} else { 
		printf("Error opening file %s\n", filename.c_str());
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

DeepSleepManager &dsm() { 
	static DeepSleepManager *firstUse = new DeepSleepManager();
	return *firstUse;
}

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

void DeepSleepManager::deepSleep(uint32_t ms) {
	prepareSleep(ms);
	OUT("DEEP SLEEP for %.2f was awake %.2fs\n", ms/1000.0, millis()/1000.0);
	esp_sleep_enable_timer_wakeup(1000LL * ms);
	fflush(stdout);
	uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
	esp_deep_sleep_start();        	
}

void DeepSleepElapsedTimer::checkInit() { 
	if (!initialized) { 
		if (getResetReason(0) != 5) {
			bootOffsetMs = startExpired ? 0xf00000 : 0;
			startTs = 0;
		}
		initialized = true;
	}
}

DeepSleepElapsedTimer::DeepSleepElapsedTimer(const string &_prefix, bool _startExpired/* = false*/) 
	: prefix(_prefix), 
	bootOffsetMs(_prefix + "_off", 0),
	startTs(_prefix + "_st", 0), 
	startExpired(_startExpired) {
	dsm().onDeepSleep([this](uint32_t ms) { prepareSleep(ms); });
}
void DeepSleepElapsedTimer::prepareSleep(uint32_t ms) { 
	bootOffsetMs = bootOffsetMs + ::millis() + typicalBootTimeMs + ms;
}
uint32_t DeepSleepElapsedTimer::millis() {
	checkInit();
	return ::millis() + bootOffsetMs - startTs;
}
void DeepSleepElapsedTimer::set(uint32_t ms) {
	checkInit(); 
	startTs = ::millis() + bootOffsetMs - ms;
}

template <typename Out>
inline void split(const std::string &s, char delim, Out result) {
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

void bin2hex(const char *in, int len, char *out, int olen) {
	len = min(len, olen / 2); 
	for (int n = 0; n < len; n++) { 
		sprintf(out + 2 * n, "%02x", in[n]);
	}
	out[2 * len] = '\0';
}

int hex2bin(const char *in, char *out, int inLength) { 
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

std::string nmeaChecksum(const std::string &s) { 
	char check = 0;
	for (const char &c : s)  
		check ^= c;
	char buf[8];
	snprintf(buf, sizeof(buf), "*%02X\n", (int)check);
	return std::string("$") + s + std::string(buf);	
}

float avgAnalogRead(int p, int avg/* = 1024*/) { 
	float bv = 0;
	pinMode(p, INPUT);
	for (int i = 0; i < avg; i++) {
#ifdef ARDUINO_ESP32S3_DEV
		if (p == 1) bv += adc1_get_raw(ADC1_CHANNEL_0);
		if (p == 2) bv += adc1_get_raw(ADC1_CHANNEL_1);
		if (p == 3) bv += adc1_get_raw(ADC1_CHANNEL_2);
		if (p == 4) bv += adc1_get_raw(ADC1_CHANNEL_3);
#else
		bv += analogRead(p);
#endif
	}
	return bv / avg;
}

String buf2str(const byte *buf, int len) { 
	String s;
	for (int i = 0; i < len; i++) {
	  s += (char)buf[i];
	}
	return s;
}
  
#include "espNowMux.h"
extern JStuff j;
void wifiDisconnect() {
	if (ESPNowMux::Instance != NULL)
		ESPNowMux::Instance->stop();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    j.jw.enabled = false;
}

bool wifiConnect() { 
    wifiDisconnect(); 
	printf("Connecting...\n");
    j.jw.enabled = true;
    j.mqtt.active = false;
    j.jw.onConnect([](){});
	j.jw.firstRun = j.jw.firstConnect = false;
    j.jw.autoConnect();
    for(int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) { 
        delay(500);
        wdtReset();
    }
    String ssid = WiFi.SSID(), ip = WiFi.localIP().toString();
    printf("Connected to AP '%s', IP=%s, channel=%d, RSSI=%d\n",
        ssid.c_str(), ip.c_str(), WiFi.channel(), WiFi.RSSI());
    return WiFi.status() == WL_CONNECTED;
}

static const ledc_timer_t LEDC_LS_TIMER  = LEDC_TIMER_0;
static const ledc_mode_t LEDC_LS_MODE = LEDC_LOW_SPEED_MODE;

void LightSleepPWM::ledcLightSleepSetup(int p, ledc_channel_t c) {
	pin = p;
	chan = c;

	ledc_channel_config_t ledc_channel = {
		.gpio_num = pin,
		.speed_mode = LEDC_LS_MODE,
		.channel = chan,
		.timer_sel = LEDC_LS_TIMER,
		.duty = 0,
		.hpoint = 0,
	};
	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LS_MODE,          // timer mode
		.duty_resolution = LEDC_TIMER_6_BIT, // resolution of PWM duty
		.timer_num = LEDC_LS_TIMER,          // timer index
		.freq_hz = 25000,                      // frequency of PWM signal
		.clk_cfg = LEDC_USE_RTC8M_CLK,       // Force source clock to RTC8M
	};

	ledc_channel.gpio_num = pin;
	ledc_channel.channel = chan;

	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_channel);
	//printf("Frequency %u Hz\n", ledc_get_freq(LEDC_LS_MODE, LEDC_LS_TIMER));
}

void LightSleepPWM::ledcLightSleepSet(int i) { 
	ledc_set_duty(LEDC_LS_MODE, chan, i);
	ledc_update_duty(LEDC_LS_MODE, chan);
#if SOC_PM_SUPPORT_RTC_PERIPH_PD || defined(CSIM)
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
#else
#warning chip does not support light sleep
#endif
	delay(100);
	//printf("Frequency %u Hz duty %d\n", 
	//    ledc_get//_freq(LEDC_LS_MODE, LEDC_LS_TIMER),
	//    ledc_get_duty(LEDC_LS_MODE, chan));
}
int LightSleepPWM::getDuty() { return ledc_get_duty(LEDC_LS_MODE, chan); }     


bool HzTimer::hz(float h) {
	bool rval = force || (millis() - last) > 1000.0 / h;
		if (rval) {
			last = millis();
			force = false;
		}
	return rval;
}

struct SimulatedFailureManager::FailSpec {
	const string name;
	float chance, duty, period, start;
};	
SimulatedFailureManager::SimulatedFailureManager() : ms("/simFail.ms") {}

void SimulatedFailureManager::addFailure(const string &name, float chance, float duty, float period, float start/*=0*/) { 
	FailSpec fs = {.name = name, .chance = chance, .duty = duty, .period = period, .start = start};
	failList.push_back(fs);
}
void SimulatedFailureManager::addFailure(const string &spec) { 
	float chance = 0, duty = 1.0, period = 60, start = 0;
	float periodMin = 0, startMin = 0;
	vector<string> words = split(spec, '=');
	if (words.size() != 2) { 
		OUT("bad fail spec '%s'", spec.c_str());
		return;
	} 
	string name = words[0];
	sscanf(words[1].c_str(),"%f,%f,%f,%f", &chance, &duty, &period, &start);
	sscanf(words[1].c_str(),"%f,%f,%f,%f", &chance, &duty, &periodMin, &startMin);
	if (periodMin > 0) period = periodMin * 60;
	if (startMin > 0) startMin = startMin * 60;
	addFailure(name, chance, duty, period, start);
}
bool SimulatedFailureManager::fail(const string &n) { 
	float now = ms.elapsed() / 1000.0;
	for(auto f: failList) { 
		if(f.name == n) {
			if (now < f.start) 
				continue;
			float x = (now - f.start) / f.period;
			if (x - floor(x) > f.duty) 
				continue;
			if (rand() / (RAND_MAX + 1.0) < f.chance) 
				return true;
		}	
	}	
	return false;
}

SimulatedFailureManager &simFailures() { 
	static SimulatedFailureManager *firstStaticUse = new SimulatedFailureManager();
	return *firstStaticUse;
}

string floatRemoveTrailingZeros(string &s) {
    using std::regex;
#ifdef CSIM // burns up too much time in simulation
	return s;
#endif
	//return s;
    //s = regex_replace(s, regex("[.]*[0]+}"), "}");
    //s = regex_replace(s, regex("[.]*[0]+,"), ",");
    //s = regex_replace(s, regex("[.]*[0]+]"), "]");
    s = regex_replace(s, regex("([0-9])[.][0]+([\" ,}])"), "$1$2");
    s = regex_replace(s, regex("([0-9][.][0-9]*[1-9])[0]+([\" ,}])"), "$1$2");
    //s = regex_replace(s, regex("[.][0]+\""), "\"");
    //s = regex_replace(s, regex("[.][0]+,"), ",");
    return s;
}


void JStuff::	run() { 
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
void JStuff::begin() { 
	beginRan = true;
	wdtInit(30);
	SPIFFSVariableESP32Base::begin();

	Serial.begin(115200);
	Serial.printf("BOOT %s git:" GIT_VERSION " mac:%s time:%05.3fs rst rsn: %d\n", 
		basename_strip_ext(__BASE_FILE__).c_str(), getMacAddress().c_str(), millis() / 1000.0, 
		getResetReason());
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
void JStuff::out(const string &s) { 
	mqtt.pub(s.c_str());
	printf("%s", s.c_str());
	printf("\n");
	jw.udpDebug(s.c_str());
}
void JStuff::out(const char *format, ...) { 
	va_list args;
	va_start(args, format);
	string s = vsfmt(format, args);
	va_end(args);
	out(s);
}
void JStuff::log(int ll, const char *format, ...) { 
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

JimWiFi::JimWiFi() { 
	aps = {	
		{"Ping-582B", "", ""},
		{"ChloeNet", "niftyprairie7", ""},
		{"MOF-Guest", "", ""},
		{"ClemmyNet","clementine is a cat", "192.168.68.138"},
		{"ChloeNet4", "niftyprairie7", ""},
		{"Station 54", "Local1747", ""},
		{"Station 72", "Local1747", ""},
	};
}

bool JimWiFi::waitConnected(int ms, int bestMatch/* = -1*/) {
	uint32_t startMs = millis();
	while(millis() - startMs < ms) {
		if (WiFi.status() == WL_CONNECTED) {
			printf("Connected\n");
			if (bestMatch >= 0) lastAP = bestMatch;
			return true;
		}
		wdtReset();
		delay(100);
	}
	return false;
}

const char *JimWiFi::getMqttServer() {
	if (lastAP >= 0) return aps[lastAP].mqtt;
	return NULL;
}

void JimWiFi::autoConnect() {
	if (!enabled)
		return;
	//WiFi.disconnect(true);
	//WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);
	//delay(100);

	int bestMatch = lastAP;
	if (bestMatch >= 0 && bestMatch < aps.size()) {
	Serial.printf("Trying cached WiFi AP '%s'...\n", aps[bestMatch].name);
	WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
	if (waitConnected(8000)) return;
		//WiFi.disconnect();
		//WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
		//if (waitConnected(2000)) return;
	}
	bestMatch = -1;

	Serial.println("Scanning...");
	WiFi.disconnect(true);
	//WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);
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
	//WiFi.disconnect();
	delay(100);
	WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
	waitConnected(20000, bestMatch);
	//if (waitConnected(20000, bestMatch)) return;
	//WiFi.disconnect();
	//WiFi.begin(aps[bestMatch].name, aps[bestMatch].pass);
	//waitConnected(12000, bestMatch);
}
