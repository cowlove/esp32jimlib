#ifndef ESPNOWMUX_H
#define ESPNOWMUX_H
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#ifndef CSIM
#include <esp_now.h>
#endif

#include "jimlib.h"
#include "callbackWrapper.h"

class BeaconSynchronizedWakeup { 
    struct PrivData;
    PrivData *priv;
public:
    void begin(int beaconCount = 3, int beaconPeriodMin = 30);
    float getSleepSec();
    int getRxCount();
    string getStats();
    void printPkts();
};


class ESPNowMux { 
    uint32_t lastSend = 0;
public:
    BeaconSynchronizedWakeup bwakeup;
    int defaultChannel = 1;
    bool pending = false;
    uint32_t lastReceiveUs = 0;
    bool alwaysBroadcast = false;
    void stop();
    bool firstInit = true;
    void check();
    void onRecv(const uint8_t *mac, const uint8_t *data, int len);
    void registerReadCallback(const char *prefix, std::function<void(const uint8_t *mac, const uint8_t *data, int len)> cb);
    void send(const char *prefix, const uint8_t *buf, int n, int tmo = 100);
    //static ESPNowMux *Instance;
    ESPNowMux();
private:
    uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t broadcastPeerInfo;
    struct cbInfo {
        esp_now_peer_info_t peerInfo;
        string prefix;
        bool dataReceived = false;
        std::function<void(const uint8_t *mac, const uint8_t *data, int len)> callback;
    };
    vector<cbInfo> callbacks;
    bool initialized = false;

    esp_now_recv_cb_t recvCbWrapper = NULL;
    esp_now_send_cb_t sendCbWrapper = NULL;

};



extern ESPNowMux defaultEspNowMux;

#ifdef CSIM
// Module encapsulating a private csim context and a private 
// instance of an espNowMux to simulate esp now traffic between
// separate ESP32s.  
class Csim_privateContext : public ESPNOW_csimOneProg { 
	ESPNOW_csimOneProg privEspNow;
	CsimContext privContext;
protected:
	ESPNowMux privMux;
public:
	Csim_privateContext(uint64_t mac = 0xddeeffddeef0) { 
		privContext.mac = mac;
		privContext.espnow = &privEspNow;
		this->context = &privContext;
        // set private context for derivived class constructors and member constructors. 
        // The end of the derived-class constructors MUST restore currentContext = &defaultContext,
        // checked by asserts() throughout esp32csim.cpp
        currentContext = this->context;    
	}
};
#endif

#endif // ESPNOWMUX_H
