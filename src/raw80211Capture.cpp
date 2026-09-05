#include "raw80211Capture.h"

#ifdef CSIM
#include "esp32csim.h"

void CsimWifiBeaconCaptureSource::inject(const WifiBeaconPacket &packet) {
    if (!running || callback == nullptr || context == nullptr || context->sleeping)
        return;
    stats.callbackFrames++;
    if (packet.data == nullptr || packet.length < 24) {
        stats.shortFrames++;
        return;
    }
    if ((packet.data[0] & 0x0c) != 0x00) {
        stats.nonManagementFrames++;
        return;
    }
    if ((packet.data[0] & 0xfc) != 0x80) {
        stats.nonBeaconFrames++;
        return;
    }
    stats.beaconFrames++;
    CsimContext *previous = currentContext;
    currentContext = context;
    callback(packet, callbackArg);
    stats.deliveredFrames++;
    currentContext = previous;
}

#else
#include <esp_wifi.h>
#include <esp_timer.h>

Esp32WifiBeaconCaptureSource *Esp32WifiBeaconCaptureSource::activeSource = nullptr;

void Esp32WifiBeaconCaptureSource::receive(void *buffer,
                                 wifi_promiscuous_pkt_type_t packetType) {
    Esp32WifiBeaconCaptureSource *source = activeSource;
    if (source == nullptr || !source->running || source->callback == nullptr)
        return;

    source->stats.callbackFrames++;

    const wifi_promiscuous_pkt_t *received =
        static_cast<const wifi_promiscuous_pkt_t *>(buffer);
    if (received->rx_ctrl.sig_len < 24) {
        source->stats.shortFrames++;
        return;
    }
    if (packetType != WIFI_PKT_MGMT) {
        source->stats.nonManagementFrames++;
        return;
    }
    if ((received->payload[0] & 0xfc) != 0x80) {
        source->stats.nonBeaconFrames++;
        return;
    }
    source->stats.beaconFrames++;
    WifiBeaconPacket packet;
    packet.driverTimestampUsec = received->rx_ctrl.timestamp;
    packet.localTimestampUsec = esp_timer_get_time();
    packet.rssi = received->rx_ctrl.rssi;
    packet.channel = received->rx_ctrl.channel;
    packet.packetType = static_cast<uint8_t>(packetType);
    packet.data = received->payload;
    packet.length = received->rx_ctrl.sig_len;
    source->callback(packet, source->callbackArg);
    source->stats.deliveredFrames++;
}

void Esp32WifiBeaconCaptureSource::start() {
    stats.callbackFrames = 0;
    stats.nonManagementFrames = 0;
    stats.shortFrames = 0;
    stats.nonBeaconFrames = 0;
    stats.beaconFrames = 0;
    stats.deliveredFrames = 0;
    activeSource = this;
    running = true;
    wifi_promiscuous_filter_t filter = {WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&Esp32WifiBeaconCaptureSource::receive);
    esp_wifi_set_promiscuous(true);
}

void Esp32WifiBeaconCaptureSource::stop() {
    if (activeSource != this)
        return;
    running = false;
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    activeSource = nullptr;
}
#endif
