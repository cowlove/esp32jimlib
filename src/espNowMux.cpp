#ifdef ESP32
#include "espNowMux.h"
#include "simulatedFailures.h"
#include "serialLog.h"
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
  
// TODO replace these with dynamic wrapper use inside ESPNowMux class
//void ESPNowMuxOnRecv(const uint8_t * mac, const uint8_t *in, int len) { 
//    ESPNowMux::Instance->onRecv(mac, in, len);
//}
//void ESPNowMuxOnSend(const uint8_t *mac_addr, esp_now_send_status_t s) { 
//   ESPNowMux::Instance->pending = false;
   //Serial.printf("ESPNowMuxOnSend: %08d %d\n", millis(), s);
//}
#if ESP_ARDUINO_VERSION_MAJOR == 3 
//void ESPNowMuxOnRecv_v3(const esp_now_recv_info *info, const uint8_t *in, int len) { 
//    ESPNowMuxOnRecv(info->src_addr, in, len);
//}
#endif

ESPNowMux defaultEspNowMux;
//ESPNowMux *ESPNowMux::Instance = NULL; // TODO replace this with static onFirstUse() wrapper

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
        WiFi.mode(WIFI_STA);
        esp_wifi_start();
        esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
        esp_now_deinit();
        esp_now_init();
        esp_now_register_recv_cb(recvCbWrapper);
        esp_now_register_send_cb(sendCbWrapper);
        memcpy(broadcastPeerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
        broadcastPeerInfo.channel = chan;  
        broadcastPeerInfo.encrypt = false;
        esp_now_add_peer(&broadcastPeerInfo);
        initialized = true;
        pending = false;
        if (firstInit) {
            //Serial.printf("ESPNowClient: using WiFi channel %d\n", chan);
            bwakeup.begin();
            firstInit = false;
        }
        esp_wifi_set_promiscuous(1);
    }
}

ESPNowMux::ESPNowMux() { 
#if ESP_ARDUINO_VERSION_MAJOR == 3     
    recvCbWrapper = CallbackWrapper<void,const esp_now_recv_info *, const uint8_t *, int>::wrap(
        [this](const esp_now_recv_info *info, const uint8_t *data, int len) { 
            this->onRecv(info->src_addr, data, len);
    });
#else
    recvCbWrapper = CallbackWrapper<void,const uint8_t *, const uint8_t *, int>::wrap(
     [this](const uint8_t *mac, const uint8_t *data, int len) { 
         this->onRecv(mac, data, len);
 });
#endif
    sendCbWrapper = CallbackWrapper<void,const uint8_t *, esp_now_send_status_t>::wrap(
        //(std::function<void(const uint8_t *, esp_now_send_status_t)>)
        [this](const uint8_t *, esp_now_send_status_t) { 
            this->pending = false;
    });
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
    esp_wifi_set_promiscuous(0);
    initialized = false;
    for(auto i = callbacks.begin(); i != callbacks.end(); i++) { 
        i->dataReceived = false;
        memcpy(i->peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
    }
    esp_now_deinit();
}


#include "callbackWrapper.h"
#ifndef CSIM
#include "esp_wifi.h"
#endif

typedef struct {
    unsigned protocol:2;
    unsigned type:2;
    unsigned subtype:4;
    unsigned ignore1:24;
    unsigned long long recv_addr:48; 
    unsigned long long send_addr:48; 
    unsigned ignore2:16;
    uint64_t timestamp;
} raw_beacon_packet_t;    

// recv addr is garbled - getting 300fb494ffff for MAC 94:B4:0F:30:34:B1  
// maybe the missing bits differentiate access points
// right now 
struct BeaconPktInfo { 
    uint64_t ssid = 0;
    int rssi;
    uint64_t ts;
    int count = 0;
    uint64_t seen;
    uint64_t seen2;
};


struct BeaconSynchronizedWakeup::PrivData { 
    string stats;
    SPIFFSVariable<std::map<uint64_t,int>> ssidCountMap = SPIFFSVariable<std::map<uint64_t,int>>("/ssidCountMap", std::map<uint64_t,int>());
    int rxCount = 0;
    BeaconPktInfo pktLog[32] = {0}; 
    raw_beacon_packet_t rawLog[32] = {0};   
    void rxInterrupt(void *buf, wifi_promiscuous_pkt_type_t type) { 
        uint64_t seen2 = micros();
        rxCount++;   
        const wifi_promiscuous_pkt_t *pt = (wifi_promiscuous_pkt_t*)buf; 
        const raw_beacon_packet_t *pk = (raw_beacon_packet_t*)pt->payload;
        if (pk->subtype == 0x8) {
            int i = 0, minI = 0;
            for(i = 0; i < (sizeof(pktLog)/sizeof(pktLog[0])); i++) {
                if (pktLog[i].ssid == pk->send_addr || pktLog[i].ssid == 0)
                    break;
                if (pktLog[i].count < pktLog[minI].count)
                    minI = i;
            }
            if (i == sizeof(pktLog)/sizeof(pktLog[0])) 
                i = minI;
            pktLog[i].ssid = pk->send_addr;
            pktLog[i].seen2 = seen2;
            pktLog[i].seen = pt->rx_ctrl.timestamp;
            pktLog[i].rssi = pt->rx_ctrl.rssi;
            pktLog[i].count++;
            pktLog[i].ts = pk->timestamp;
        }    
    }
    int pktIndex = 0;
    void rxInterruptLogPackets(void *buf, wifi_promiscuous_pkt_type_t type) { 
        uint64_t seen2 = micros();
        const wifi_promiscuous_pkt_t *pt = (wifi_promiscuous_pkt_t*)buf; 
        const raw_beacon_packet_t *pk = (raw_beacon_packet_t*)pt->payload;
        if (pk->subtype != 0x8) 
            return;

        rxCount++;   
        int i = pktIndex++;
        if (i >= sizeof(rawLog)/sizeof(rawLog[0]))
            return;
        rawLog[i] = *pk;
    }
    vector<BeaconPktInfo> getResults() { 
        std::map<uint64_t,int> countMap = ssidCountMap.read();
        if (getResetReason() != 5) countMap.clear();
        // tmp don't use countMap
        countMap.clear();

        vector<BeaconPktInfo> r;
        for(int i = 0; i < sizeof(pktLog)/sizeof(pktLog[0]); i++) {
            if (pktLog[i].count == 0)
                continue; 
            if (countMap.find(pktLog[i].ssid) != countMap.end()) 
                pktLog[i].count += countMap[pktLog[i].ssid]; 
            r.push_back(pktLog[i]);
        }
        
        //countMap.clear();
        for(auto i : r) countMap[i.ssid] = i.count;

        // limit stored ssid count map size, trim off lowest counts
        const int maxSize = 20;
        if (countMap.size() > maxSize) {
            vector<int> counts;
            for(auto i : countMap) counts.push_back(i.second);
            sort(counts.begin(), counts.end());
            int countThreshold = counts[maxSize];
            std::map<uint64_t,int> newMap;
            for(auto i : countMap) if(i.second > countThreshold) newMap[i.first] = i.second;
            countMap = newMap; 
            // TODO limit size of map, remove smallest counts if size > max
        }
        ssidCountMap = countMap;
        for(auto b : r) { 
            printf("RAW     %" PRIx64 " %2d %" PRIx64 "\n", b.ssid, b.count, b.ts);
        }
        return r;
    }

    float getBest(int best, uint64_t clock) {
        vector<BeaconPktInfo> r = getResults(); 
        sort(r.begin(), r.end(), [](const BeaconPktInfo &a, const BeaconPktInfo &b) { return a.count > b.count; });
        for(auto b : r) { 
            printf("SORTED  %" PRIx64 " %2d %" PRIx64 "\n", b.ssid, b.count, b.ts);
        }

        // only keep the top ssids that account for 80% of counts
        int completeSum = 0, sum = 0;
        for(auto b : r) completeSum += b.count;
        OUT("completeSum %d", completeSum);
        for(auto i = r.begin(); i != r.end(); i++) { 
            sum += i->count;
            if (sum > completeSum * 1.00) { 
                r.erase(i + 1, r.end());
                break;
            }
        }

        // trim off everything that is less that 20% of the best beacon
        int maxCount = 0;
        for(auto b : r) maxCount = max(maxCount, b.count);
        for(auto i = r.begin(); i != r.end(); i++) { 
            if (i->count < maxCount * 0.0) {
                r.erase(i, r.end());
                break;
            }
        }

        // trim to top #best number of results
        if (best > 0 && r.size() > best) 
            r.erase(r.begin() + best, r.end());
        for(auto b : r) { 
            printf("TRIMMED %" PRIx64 " %2d %" PRIx64 "\n", b.ssid, b.count, b.ts);
        }
        sort(r.begin(), r.end(), [clock](const BeaconPktInfo &a, const BeaconPktInfo &b) { 
            return a.ts % clock > b.ts % clock; });
        for(auto b : r) { 
            printf("SOONEST %" PRIx64 " %2d %" PRIx64 "  %.02f\n", b.ssid, b.count, b.ts, (clock - (b.ts % clock)) / 1000000.0 );
        }
    
        if (r.size() > 0) { 
            float rval = (clock - (r[0].ts % clock)) / 1000000.0;
            printf("\nBEST*** %" PRIx64 " %2d %" PRIx64 "  %.02f\n\n\n", r[0].ssid, r[0].count, r[0].ts, rval);
            stats = sfmt("mac %" PRIx64 " count %d rssi %d", r[0].ssid, r[0].count, r[0].rssi);            
            return rval - (micros() - r[0].seen2) / 1000000.0;
        } else {
            stats = "mac none count 0"; 
            return -1;
        }
    }
};

void BeaconSynchronizedWakeup::begin(int beaconCount /*defaulted*/, int beaconPeriodMin /*defaulted*/) {
    priv = new PrivData();
    bzero(priv->pktLog, sizeof(priv->pktLog));
#if 0 
    esp_wifi_stop();
    esp_wifi_deinit();
    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
    my_config.ampdu_tx_enable = 0;
    esp_wifi_init(&my_config);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_disconnect();
#endif
    esp_wifi_set_promiscuous(1);
    wifi_promiscuous_filter_t filter = {WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(CallbackWrapper<void, void *, wifi_promiscuous_pkt_type_t>::wrap(
        [this](void *buf, wifi_promiscuous_pkt_type_t type) { 
            priv->rxInterrupt(buf, type);
            if(false) // enable to use printPkts() below 
                priv->rxInterruptLogPackets(buf, type);
    }));
}

int BeaconSynchronizedWakeup::getRxCount() { 
    return priv->rxCount;
}
float BeaconSynchronizedWakeup::getSleepSec() {
    return priv->getBest(-1, 1000000 * 60 * 30); 
}

string BeaconSynchronizedWakeup::getStats() { 
    return priv->stats;
}

void BeaconSynchronizedWakeup::printPkts() { 
    for(int i = 0; i < sizeof(priv->rawLog)/sizeof(priv->rawLog[0]); i++) {
        raw_beacon_packet_t &b = priv->rawLog[i];
        if (b.send_addr == 0)
            continue; 
        OUT("%08" PRIx64 " %08" PRIx64, b.send_addr, b.timestamp);
        uint8_t *p = (uint8_t *)&b;
        for(int b = 0; b < sizeof(raw_beacon_packet_t); b++) { 
            printf("%02x", p[b]);
            if (b % 8 == 7) printf(" ");
            if (b % 32 == 31) printf("\n");
        }
        printf("\n");
    }
}
#endif // ESP32
