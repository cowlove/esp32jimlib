#ifndef RAW80211CAPTURE_H
#define RAW80211CAPTURE_H

#include <stddef.h>
#include <stdint.h>

// Portable view of one promiscuously received 802.11 frame.  data is valid
// only for the duration of the callback; consumers that defer processing must
// copy it into their own bounded queue.
struct Raw80211Packet {
    uint64_t timestampUsec = 0;
    int8_t rssi = 0;
    uint8_t channel = 0;
    uint8_t packetType = 0;
    const uint8_t *data = nullptr;
    size_t length = 0;
};

using Raw80211Callback = void (*)(const Raw80211Packet &, void *);

#ifdef CSIM

struct CsimContext;

// Per-device synthetic capture endpoint.  The simulated radio/environment
// calls inject(); application code uses the same setCallback/start/stop API as
// it does on hardware.
class CsimCaptureSource {
    CsimContext *context = nullptr;
    Raw80211Callback callback = nullptr;
    void *callbackArg = nullptr;
    bool running = false;
public:
    explicit CsimCaptureSource(CsimContext *owner = nullptr) : context(owner) {}
    void bind(CsimContext *owner) { context = owner; }
    void setCallback(Raw80211Callback cb, void *arg = nullptr) {
        callback = cb;
        callbackArg = arg;
    }
    void start() { running = true; }
    void stop() { running = false; }
    bool active() const { return running; }
    void inject(const Raw80211Packet &packet);
};

#else

#include <esp_wifi.h>

// The ESP-IDF promiscuous receiver is process/device-global.  There is only
// one HardwareContext on an ESP32, so this concrete source owns that binding.
class Esp32CaptureSource {
    Raw80211Callback callback = nullptr;
    void *callbackArg = nullptr;
    bool running = false;

    static Esp32CaptureSource *activeSource;
    static void receive(void *buffer, wifi_promiscuous_pkt_type_t packetType);
public:
    void setCallback(Raw80211Callback cb, void *arg = nullptr) {
        callback = cb;
        callbackArg = arg;
    }
    void start();
    void stop();
    bool active() const { return running; }
};

#endif

#endif
