#include "espNowMux.h"

#ifndef CSIM
#include "WiFiServer.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_private/wifi.h>
#endif

void printMac(const uint8_t *x) {
    Serial.printf("%08x=%02x:%02x:%02x:%02x:%02x:%02x ", 
              x, x[0], x[1], x[2], x[3], x[4], x[5]);
}
  

void ESPNowMuxOnRecv(const uint8_t * mac, const uint8_t *in, int len) { 
    ESPNowMux::Instance->onRecv(mac, in, len);
}
void ESPNowMuxOnSend(const uint8_t *mac_addr, esp_now_send_status_t s) { 
   ESPNowMux::Instance->pending = false;
   //Serial.printf("ESPNowMuxOnSend: %08d %d\n", millis(), s);
}
#if ESP_ARDUINO_VERSION_MAJOR == 3 
void ESPNowMuxOnRecv_v3(const esp_now_recv_info *info, const uint8_t *in, int len) { 
    ESPNowMuxOnRecv(info->src_addr, in, len);
}
#endif

ESPNowMux espNowMux;
ESPNowMux *ESPNowMux::Instance = NULL;

void ESPNowMux::check() {
    if (!initialized) { 
        int chan = WiFi.channel();
        if (!WiFi.isConnected()) { 
            chan = defaultChannel;
            esp_wifi_stop();
            esp_wifi_deinit();
            wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
            my_config.ampdu_tx_enable = 0;
            esp_wifi_init(&my_config);
            esp_wifi_start(); 
        }
        if (firstInit) {
            //Serial.printf("ESPNowClient: using WiFi channel %d\n", chan);
        }
        firstInit = false;
        WiFi.mode(WIFI_STA);
        esp_wifi_start();
        esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
        esp_now_deinit();
        esp_now_init();
    #if ESP_ARDUINO_VERSION_MAJOR == 3 
        esp_now_register_recv_cb(ESPNowMuxOnRecv_v3);
    #else
        esp_now_register_recv_cb(ESPNowMuxOnRecv);
    #endif
        esp_now_register_send_cb(ESPNowMuxOnSend);
        memcpy(broadcastPeerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
        broadcastPeerInfo.channel = chan;  
        broadcastPeerInfo.encrypt = false;
        esp_now_add_peer(&broadcastPeerInfo);
        initialized = true;
        pending = false;
    }
}
void ESPNowMux::onRecv(const uint8_t *mac, const uint8_t *data, int len) {
    lastReceiveUs = micros(); 
    for(auto i = callbacks.begin(); i != callbacks.end(); i++) { 
        int prefixLen = i->prefix.length();
        if (len > prefixLen && memcmp(data, i->prefix.c_str(), i->prefix.length()) == 0) {
            if (i->dataReceived == false) { 
                i->dataReceived = true;
                memcpy(i->peerInfo.peer_addr, mac, sizeof(i->peerInfo.peer_addr));
                esp_now_add_peer(&i->peerInfo);
            } 
            //Serial.printf("EspNOWMux::onRecv(%d)\n", len);
            i->callback(mac, data + prefixLen, len - prefixLen);
        }
    }
}
void ESPNowMux::registerReadCallback(const char *prefix, std::function<void(const uint8_t *mac, const uint8_t *data, int len)> cb) {
    cbInfo i;
    i.prefix = prefix;
    i.callback = cb;
    i.dataReceived = false;
    check();
    memcpy(&i.peerInfo, &broadcastPeerInfo, sizeof(broadcastPeerInfo));
    callbacks.push_back(i);
}

void ESPNowMux::send(const char *prefix, const uint8_t *buf, int n, int tmo /*= 100*/) {
    check();
    if (SIMFAILURE("espnow-tx") || SIMFAILURE("espnow"))
        return;
    uint8_t *mac = broadcastAddress;
    for(auto i = callbacks.begin(); i != callbacks.end(); i++) { 
        if (!alwaysBroadcast && strcmp(prefix, i->prefix.c_str()) == 0)
        mac = i->peerInfo.peer_addr;
    }
    size_t sent = 0;
    while(sent < n) { 
        uint8_t txBuf[200];
        int pl = min((size_t)(sizeof(txBuf) - strlen(prefix)), n - sent);
        //if (pl > 150) { // throttle espNOW for reliability 
        //  while(millis() - lastSend < 30) delay(1);
        //}
        memcpy(txBuf, prefix, strlen(prefix));
        memcpy(txBuf + strlen(prefix), buf + sent, pl);
        int t = tmo;
        while(pending && t-- > 0) {
            //Serial.println("delaying");
            delay(1);
        }
        if (pending == false) {
            pending = true;
            int r = esp_now_send(mac, txBuf, pl + strlen(prefix));
            //Serial.printf("esp_now_send(): %08d %d\n", millis(), pl);
            if (r != 0) {
                Serial.printf("esp_now_send(%02x:%02x:%02x:%02x:%02x:%02x, %d) error %d\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], pl + strlen(prefix), r);
            }
            sent += pl;
        } else {
            //Serial.printf("ESPNowMux::write timeout(), closing\n");
            stop();
            return;
        }
        lastSend = millis();
    } 
} 

void ESPNowMux::stop() { 
    initialized = false;
    for(auto i = callbacks.begin(); i != callbacks.end(); i++) { 
        i->dataReceived = false;
        memcpy(i->peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
    }
    esp_now_deinit();
}
