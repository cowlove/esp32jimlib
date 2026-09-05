#include "raw80211Capture.h"

#ifdef CSIM
#include "esp32csim.h"

void CsimCaptureSource::inject(const Raw80211Packet &packet) {
    if (!running || callback == nullptr || context == nullptr || context->sleeping)
        return;
    CsimContext *previous = currentContext;
    currentContext = context;
    callback(packet, callbackArg);
    currentContext = previous;
}

#else
#include <esp_wifi.h>

Esp32CaptureSource *Esp32CaptureSource::activeSource = nullptr;

void Esp32CaptureSource::receive(void *buffer,
                                 wifi_promiscuous_pkt_type_t packetType) {
    Esp32CaptureSource *source = activeSource;
    if (source == nullptr || !source->running || source->callback == nullptr)
        return;

    const wifi_promiscuous_pkt_t *received =
        static_cast<const wifi_promiscuous_pkt_t *>(buffer);
    Raw80211Packet packet;
    packet.timestampUsec = received->rx_ctrl.timestamp;
    packet.rssi = received->rx_ctrl.rssi;
    packet.channel = received->rx_ctrl.channel;
    packet.packetType = static_cast<uint8_t>(packetType);
    packet.data = received->payload;
    packet.length = received->rx_ctrl.sig_len;
    source->callback(packet, source->callbackArg);
}

void Esp32CaptureSource::start() {
    activeSource = this;
    running = true;
    wifi_promiscuous_filter_t filter = {WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&Esp32CaptureSource::receive);
    esp_wifi_set_promiscuous(true);
}

void Esp32CaptureSource::stop() {
    if (activeSource != this)
        return;
    running = false;
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    activeSource = nullptr;
}
#endif
