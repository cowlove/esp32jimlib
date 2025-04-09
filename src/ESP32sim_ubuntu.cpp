#ifdef CSIM
#include "ESP32sim_ubuntu.h"

uint64_t _micros = 0;
uint64_t _microsMax = 0xffffffff;


int Semaphores[10];
int nextSem = 0;
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



int ESP32sim_currentPwm[16];
void ledcWrite(int chan, int val) {
		ESP32sim_currentPwm[chan] = val;
} 


ESP32sim_pinManager pinManObject;
ESP32sim_pinManager *ESP32sim_pinManager::manager = &pinManObject;


uint64_t csim_mac = 0xffeeddaabbcc;
void esp_read_mac(uint8_t *out, int) {
	uint64_t nmac = htonll(csim_mac);
	uint8_t *p = (uint8_t *)&nmac;
	memcpy(out, p + 2, 6);
}


FakeSerial Serial, Serial1, Serial2;
InterruptManager intMan;
FakeESP ESP;
FakeArduinoOTA ArduinoOTA;
FakeSPIFFS SPIFFS, LittleFS;
ESP32sim &esp32sim() { 
	static ESP32sim *staticFirstUse = new ESP32sim();
	return *staticFirstUse;
}
FakeWiFi WiFi;
FakeSD SD;
FakeWire Wire;
Update_t Update;
FakeCAN CAN;
DHT::Csim DHT::csim;
ESP32sim_flags csim_flags;

void ESP32sim_exit() { 	esp32sim().exit(); }

uint64_t sleep_timer = 0;
vector<deepSleepHookT> deepSleepHooks;
void onDeepSleep(deepSleepHookT func) { deepSleepHooks.push_back(func); }
void esp_deep_sleep_start() {
	double newRunSec = -1;
	if (esp32sim().seconds >= 0) {
		newRunSec = esp32sim().seconds - (sleep_timer + _micros) / 1000000;
		if (newRunSec < 0) { 
			fflush(stdout);
			ESP32sim_exit();
		}
	}

	for (auto i : deepSleepHooks) i(sleep_timer);
	char *argv[128];
	int argc = 0; 
	// strip out all --boot-time, --seconds, --reset-reason and --show-args command line arguments
	for(char *const *p = esp32sim().argv; *p != NULL; p++) {
		if (strcmp(*p, "--boot-time") == 0) p++;
		else if (strcmp(*p, "--seconds") == 0) p++;
		else if (strcmp(*p, "-s") == 0) p++;
		else if (strcmp(*p, "--reset-reason") == 0) p++;
		else if (strcmp(*p, "--show-args") == 0) {/*skip arg*/}
		else argv[argc++] = *p;
	}
	char bootTimeBuf[32], secondsBuf[32];
	snprintf(bootTimeBuf, sizeof(bootTimeBuf), "%ld", esp32sim().bootTimeUsec + sleep_timer + _micros);
	argv[argc++] = (char *)"--boot-time";
	argv[argc++] = bootTimeBuf; 
	snprintf(secondsBuf, sizeof(secondsBuf), "%f", newRunSec);
	argv[argc++] = (char *)"--seconds";
	argv[argc++] = secondsBuf; 
	argv[argc++] = (char *)"--reset-reason";
	argv[argc++] = (char *)"5";
	argv[argc++] = (char *)"--show-args";
	argv[argc++] = NULL; 
	execv("./csim", argv); 
}
void esp_light_sleep_start() {
	delayMicroseconds(0); // run csim hooks 
	_micros += sleep_timer; 
	if (esp32sim().seconds > 0 && micros() / 1000000.0 > esp32sim().seconds)
		ESP32sim_exit();
} 

ESPNOW_csimInterface *ESPNOW_sendHandler = NULL;
esp_now_send_cb_t ESP32_esp_now_send_cb = NULL;
esp_now_recv_cb_t ESP32_esp_now_recv_cb = NULL;

vector<HTTPClient::postHookInfo> HTTPClient::csim_hooks = {
	{".*", true, HTTPClient::csim_defaultOnPOST },
	{".*", false, HTTPClient::csim_defaultOnGET }};


// TODO: extend this to use vector<unsigned char> to handle binary data	
WiFiUDP::InputMap WiFiUDP::inputMap;

ESP32sim_Module::ESP32sim_Module() { 
	esp32sim().modules.push_back(this);
}

int main(int argc, char **argv) {
	esp32sim().main(argc, argv);
}


void ESP32sim::main(int argc, char **argv) {
	this->argc = argc;
	this->argv = argv;
	Serial.toConsole = true;
	int showargs = 0;
	for(char **a = argv + 1; a < argv+argc; a++) {
		if (strcmp(*a, "--serial") == 0) {
			printf("--serial is depricated, use --serialConsole\n");
			::exit(-1);
		}
		else if (strcmp(*a, "--serialConsole") == 0) sscanf(*(++a), "%d", &Serial.toConsole); 
		else if (strcmp(*a, "--wifi-errors") == 0) sscanf(*(++a), "%d", &WiFi.simulatedFailMinutes); 
		else if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%lf", &seconds); 
		else if (strcmp(*a, "-s") == 0) sscanf(*(++a), "%lf", &seconds); 
		else if (strcmp(*a, "--boot-time") == 0) sscanf(*(++a), "%ld", &bootTimeUsec); 
		else if (strcmp(*a, "--show-args") == 0) showargs = 1; 
		else if (strcmp(*a, "--reset-reason") == 0) sscanf(*(++a), "%d", &resetReason); 
		else if (strcmp(*a, "--mac") == 0) { 
			sscanf(*(++a), "%lx", &csim_mac);
		} else if (strcmp(*a, "--espnowPipe") == 0) { 
			ESPNOW_sendHandler= new ESPNOW_csimPipe(*(++a), *(++a));
		} else if (strcmp(*a, "--espnowOneProg") == 0) { 
			ESPNOW_sendHandler= new ESPNOW_csimOneProg();
			csim_flags.OneProg = true;
		} else if (strcmp(*a, "--interruptFile") == 0) { 
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
	if (showargs) { 
		printf("args: ");
			for(char **a = argv; a < argv+argc; a++) 
				printf("%s ", *a);
		printf("\n");
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
void ESP32sim::exit() { 
	for(vector<ESP32sim_Module *>::iterator it = modules.begin(); it != modules.end(); it++) 
		(*it)->done();	
	::exit(0);
}
void ESP32sim::delayMicroseconds(long long us) { 
	do {
		int step = min(100000LL, us);
		_micros += step;
		for(auto it : modules) {
			it->loop();
		}
		intMan.run();
		us -= step;
		if (esp32sim().seconds > 0 && micros() / 1000000.0 > esp32sim().seconds)
			ESP32sim_exit();
	} while(us > 0);
}

void delayMicroseconds(int m) { esp32sim().delayMicroseconds(m); }

uint32_t micros() { return _microsMax > 0 ? ++_micros & _microsMax : ++_micros; }
uint32_t millis() { return ++_micros / 1000; }
int rtc_get_reset_reason(int) { return esp32sim().resetReason; } 

int FakeWiFi::status() {
   bool disable = simulatedFailMinutes > 0 && 
	   ((micros() + esp32sim().bootTimeUsec) / 1000000 / 60) 
	   % simulatedFailMinutes == simulatedFailMinutes - 1; 
   if (disable) curStatus = WL_DISCONNECTED; 
   return curStatus; 
} 
#endif