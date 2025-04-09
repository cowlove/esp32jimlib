#ifndef ESPNOWMUX_H
#define ESPNOWMUX_H
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#ifndef CSIM
#endif

#include "jimlib.h"

void printMac(const uint8_t *x);

void ESPNowMuxOnRecv(const uint8_t *mac, const uint8_t *in, int len);
void ESPNowMuxOnSend(const uint8_t *mac_addr, esp_now_send_status_t s);
#if ESP_ARDUINO_VERSION_MAJOR == 3 
void ESPNowMuxOnRecv_v3(const esp_now_recv_info *info, const uint8_t *in, int len) { 
    ESPNowMuxOnRecv(info->src_addr, in, len);
}
#endif

class ESPNowMux { 
    uint32_t lastSend = 0;
public:
    int defaultChannel = 1;
    bool pending = false;
    uint32_t lastReceiveUs = 0;
    bool alwaysBroadcast = false;
    ESPNowMux() { Instance = this; } // TODO static constructor order issue, 
    void stop();
    bool firstInit = true;
    void check();
    void onRecv(const uint8_t *mac, const uint8_t *data, int len);
    void registerReadCallback(const char *prefix, std::function<void(const uint8_t *mac, const uint8_t *data, int len)> cb);
    void send(const char *prefix, const uint8_t *buf, int n, int tmo = 100);
    static ESPNowMux *Instance;
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
};

extern ESPNowMux espNowMux;
void ESPNowMuxOnRecv(const uint8_t * mac, const uint8_t *in, int len);
void ESPNowMuxOnSend(const uint8_t *mac_addr, esp_now_send_status_t s);

#endif // ESPNOWMUX_H
