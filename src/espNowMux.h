#ifdef ESP32
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
#include "raw80211Capture.h"

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
    volatile uint32_t sendAttempts = 0;
    volatile uint32_t sendSuccesses = 0;
    volatile uint32_t sendFailures = 0;
    volatile uint32_t sendBusyDrops = 0;
public:
    // Keep each ESP-NOW transmission at or below this many bytes.  Although
    // ESP-NOW can support a larger vendor payload on some core versions, this
    // library deliberately uses a conservative 200-byte physical packet for
    // compatibility and reliability. send() splits larger logical payloads.
    // ReliableStream's onePacket hint records when its encoded frame plus the
    // routing prefix exceeds this limit, but the write still proceeds.
    static constexpr size_t physicalPacketBytes = 200;
    BeaconSynchronizedWakeup bwakeup;
    int defaultChannel = 1;
    bool pending = false;
    uint32_t lastReceiveUs = 0;
    bool alwaysBroadcast = false;
    void stop();
    bool firstInit = true;
    void check();
    bool writeBusy();
    uint32_t getSendAttempts() const { return sendAttempts; }
    uint32_t getSendSuccesses() const { return sendSuccesses; }
    uint32_t getSendFailures() const { return sendFailures; }
    uint32_t getSendBusyDrops() const { return sendBusyDrops; }
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
	CsimCaptureSource capture;
public:
	Csim_privateContext(uint64_t mac = 0xddeeffddeef0) : capture(&privContext) {
		privContext.mac = mac;
		privContext.espnow = &privEspNow;
		this->context = &privContext;
		// Delivery by the private transport is device work too: callbacks must
		// run under this context and must be suppressed while it is asleep.
		privEspNow.context = &privContext;
		// Keep the private context selected while derived-class constructors and
		// member constructors run.  The most-derived constructor restores the
		// default context after construction is complete.
		currentContext = this->context;
	}
};
#endif

#endif // ESPNOWMUX_H
#endif // ESP32
