#include "espNowMux.h"

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
ESPNowMux espNowMux;
ESPNowMux *ESPNowMux::Instance = NULL;
