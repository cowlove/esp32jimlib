#ifdef CSIM
#include "ESP32sim_ubuntu.h"

uint64_t _micros = 0;
uint64_t _microsMax = 0xffffffff;
#include "jimlib.h"

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

DHT::Csim &DHT::csim() { static Csim *firstUse = new Csim(); return *firstUse; }
ESP32sim_flags csim_flags;

void ESP32sim_exit() { 	esp32sim().exit(); }

uint64_t sleep_timer = 0;
vector<deepSleepHookT> deepSleepHooks;
void csim_onDeepSleep(deepSleepHookT func) { deepSleepHooks.push_back(func); }
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

static int csim_defaultOnPOST(const char *url, const char *hdr, const char *data, string &result) {
	string cmd = string("curl --silent -X POST -H '") + hdr + "' -d '" +
		data + "' " + url;
	FILE *fp = popen(cmd.c_str(), "r");
	int bufsz = 64 * 1024;
	char *buf = (char *)malloc(bufsz);
	result = fgets(buf, bufsz, fp);
	result = buf;
	fclose(fp);
	free(buf);
	return 200;
}

static int csim_defaultOnGET(const char *url, const char *hdr, const char *data, string &result) {
	string cmd = "curl --silent -X GET '" + string(url) + "'";
	FILE *fp = popen(cmd.c_str(), "r");
	int bufsz = 128 * 1024;
	char *buf = (char *)malloc(bufsz);
	int n = fread(buf, 1, bufsz, fp);
	result.assign(buf, n);
	fclose(fp);
	free(buf);
	return 200;
}

vector<HTTPClient::postHookInfo> &HTTPClient::csim_hooks() { 
	static vector<postHookInfo> *firstUse = new vector<postHookInfo>({
		{".*", true, csim_defaultOnPOST },
		{".*", false, csim_defaultOnGET }});
	return *firstUse;
}

int HTTPClient::csim_doPOSTorGET(bool isPost, const char *data, string &result) { 
	string hdr = header1 + ": " + header2;
	auto p = csim_hooks().end();
	std::cmatch m;
	for(auto i  = csim_hooks().begin(); i != csim_hooks().end(); i++) { // find best hook
		if (std::regex_match(url.c_str(), m, std::regex(i->urlRegex))) {
			if (i->urlRegex.length() > p->urlRegex.length() && i->isPost == isPost) {
				p = i;
			}
		}
	}
	if (p != csim_hooks().end()) 
		return p->func(url.c_str(), hdr.c_str(), data, result);
	return -1;
}

//vector<HTTPClient::postHookInfo> HTTPClient::csim_hooks = {
//	{".*", true, csim_defaultOnPOST },
//	{".*", false, csim_defaultOnGET }};


// TODO: extend this to use vector<unsigned char> to handle binary data	
WiFiUDP::InputMap WiFiUDP::inputMap;

void ESPNOW_csimOneProg::send(const uint8_t *mac_addr, const uint8_t *data, int data_len) {//override {
	SimPacket p; 
	memcpy(p.mac, mac_addr, sizeof(p.mac));
	const char *cp = (const char *)data;
	p.data = string(cp, cp + data_len);
	pktQueue.push_back(p);
}
void ESPNOW_csimOneProg::loop() { // override
	for(auto pkt : pktQueue) {
		if (ESP32_esp_now_recv_cb != NULL) {
			if (SIMFAILURE("espnow-rx") || SIMFAILURE("espnow"))
				continue;
			ESP32_esp_now_recv_cb(pkt.mac, (const uint8_t *)pkt.data.c_str(), pkt.data.length());
		}
	}
	pktQueue.clear();
} 

// Higher fidelity ESPNOW simulation between two processes using named pipes
//   example:
//   ./csim --espnowPipe /tmp/fifo2 /tmp/fifo1 --mac fff1
//   ./csim --espnowPipe /tmp/fifo1 /tmp/fifo2 --mac fff2

class ESPNOW_csimPipe : public ESP32sim_Module, public ESPNOW_csimInterface {
	int fdIn;
	const char *outFilename;
public:
	ESPNOW_csimPipe(const char *inFile, const char *outF);
	void send(const uint8_t *mac_addr, const uint8_t *data, int len) override;
	void loop() override;
};



ESPNOW_csimPipe::ESPNOW_csimPipe(const char *inFile, const char *outF) : outFilename(outF) { 
	fdIn = open(inFile, O_RDONLY | O_NONBLOCK); 
}

void ESPNOW_csimPipe::send(const uint8_t *mac_addr, const uint8_t *data, int len) { //override
	int fdOut = open(outFilename, O_WRONLY | O_NONBLOCK); 
	if(fdOut > 0) {
		int n = ::write(fdOut, data, len);
		if (n <= 0) { 
			printf("write returned %d", n);

		}
	}
	close(fdOut);
}

void ESPNOW_csimPipe::loop() { // override
	static uint8_t mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	char buf[512];
	int n;
	while(fdIn > 0 && (n = ::read(fdIn, buf, sizeof(buf))) > 0) { 
		if (SIMFAILURE("espnow-rx") || SIMFAILURE("espnow"))
		return;

		if (ESP32_esp_now_recv_cb != NULL) {
			ESP32_esp_now_recv_cb(mac, (uint8_t *)buf, n);
		}	
	}
} 

int esp_now_send(const uint8_t*mac, const uint8_t*data, size_t len) {
	if (SIMFAILURE("espnow-tx") || SIMFAILURE("espnow"))
		return ESP_OK;

	if (ESP32_esp_now_send_cb != NULL)
		ESP32_esp_now_send_cb(mac, ESP_NOW_SEND_SUCCESS); 
	if (ESPNOW_sendHandler != NULL) 
		ESPNOW_sendHandler->send(mac, data, len); 
	return ESP_OK; 
}


ESP32sim_Module::ESP32sim_Module() { 
	esp32sim().modules.push_back(this);
}

int main(int argc, char **argv) {
	esp32sim().main(argc, argv);
}

void ESP32sim::parseArgs(int argc, char **argv) {
	for(char **a = argv; a < argv+argc; a++) {
		if (strcmp(*a, "--serial") == 0) {
			printf("--serial is depricated, use --serialConsole\n");
			::exit(-1);
		}
		else if (strcmp(*a, "--fail") == 0) simFailures().addFailure(*(++a));
		else if (strcmp(*a, "--serialConsole") == 0) sscanf(*(++a), "%d", &Serial.toConsole); 
		else if (strcmp(*a, "--wifi-errors") == 0) sscanf(*(++a), "%d", &WiFi.simulatedFailMinutes); 
		else if (strcmp(*a, "--seconds") == 0) sscanf(*(++a), "%lf", &seconds); 
		else if (strcmp(*a, "-s") == 0) sscanf(*(++a), "%lf", &seconds); 
		else if (strcmp(*a, "--boot-time") == 0) sscanf(*(++a), "%ld", &bootTimeUsec); 
		else if (strcmp(*a, "--show-args") == 0) showArgs = true; 
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
}
void ESP32sim::main(int argc, char **argv) {
	this->argc = argc;
	this->argv = argv;
	Serial.toConsole = true;
	parseArgs(argc, argv); // skip argv[0]
	if (showArgs) { 
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

void FakeSerial::printf(const char  *f, ...) { 
	va_list args;
	va_start (args, f);
	if (toConsole) 
		vprintf(f, args);
	va_end (args);
}

int FakeSerial::available() {
	if (inputLine.length() > 0)
		return inputLine.length(); 
	if (inputQueue.size() > 0 && millis() >= inputQueue[0].first) 
		return inputQueue[0].second.length();
	return 0;
}
int FakeSerial::readBytes(uint8_t *b, int l) {
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

int FakeSerial::read() { 
	uint8_t b;
	if (readBytes(&b, 1) == 1)
		return b;
	return -1; 
}

void FakeCAN::run() { 
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

int FakeCAN::read() { 
	char *p = strchr((char *)simFileLine.c_str(), ']');
	if (p == NULL) return 0;
	for (int i = 0; i < packetByteIndex / 8 + 1; i++) {
		p = strchr(p + 1, ' ');
		if (p == NULL) return 0;
	}
	unsigned int rval;
	char b[4];
	b[0] = *(p + 1 + (packetByteIndex % 8) * 2);
	b[1] = *(p + 2 + (packetByteIndex % 8) * 2);
	b[3] = 0;
	if (sscanf(b, "%02x", &rval) != 1)
		return 0;
	packetByteIndex++;
	return rval; 
}


#endif