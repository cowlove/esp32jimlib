#ifdef ESP32
#ifndef RELIABLESTREAM_H
#define RELIABLESTREAM_H
#include <string>
#include "Arduino_CRC32.h" 
#include "reliableStreamFraming.h"
#include "reliableStreamByteQueue.h"
using std::string;

class ReliableStreamInterface {
public:
  virtual int read(string &) = 0;
  void write(const string &s) { write(s, false); }
  virtual void write(const string &, bool onePacket) = 0;
  virtual uint32_t getOnePacketMissCount() const = 0;
  virtual uint32_t getFrameLossCount() const = 0;
  virtual uint32_t getSequenceDiscontinuityCount() const = 0;
  virtual uint32_t resetCount() const = 0;
  void begin(const char *h, uint16_t p) { host = h; port = p; }
protected:
  string host;
  uint16_t port; 
};


// Historical name: ReliableStream. The design goal is best-effort, one-shot
// logical-frame delivery: framing and CRC make receive acceptance all-or-
// nothing, but there are no acknowledgements, retries, or delivery/order
// guarantees. The modulo-256 sequence is diagnostic only. "Reliable" refers
// only to resetting/reconnecting an idle channel, not reliable delivery.
//
// Frames use ~SSLLLL<escaped-payload><CRC32>. The historical "-1 ACK" payload
// is an out-of-band liveness heartbeat, not an application acknowledgement.
// write(payload, true) is only a one-physical-packet performance hint; it does
// not reject a logical write. Packet transports can count fragmentation misses.

template<class T>
class ReliableStream : public ReliableStreamInterface { 
public:
  // Fixed storage bounds memory; exceeding the configured threshold clears
  // queued bytes and drops the input chunk that crossed it.
  static constexpr size_t byteQueueCapacity = 4096;
  static constexpr size_t defaultClearThreshold = 2048;
private:
  uint32_t lastSend = 0, lastRecv = 0, lastRecon = 0;
  uint32_t clientResetCount = 0;
  uint32_t frameLossCount = 0, sequenceDiscontinuityCount = 0;
  uint8_t sendSequence = 0, receiveSequence = 0;
  bool haveReceiveSequence = false;
  static constexpr uint32_t initialInboundTimeoutMs = 3000;
  static constexpr uint32_t maxInboundTimeoutMs = 30000;
  uint32_t inboundTimeoutMs = initialInboundTimeoutMs;
  const string HEARTBEAT_pkt = "-1 ACK\n";
  ReliableStreamByteQueue<byteQueueCapacity> readBuffer;
public:
  ReliableStream() : readBuffer(defaultClearThreshold) {}
  ReliableStream(const char *s, uint16_t p,
      size_t upperClearThreshold = defaultClearThreshold)
      : readBuffer(upperClearThreshold) { begin(s, p); }
  void write(const string &s, bool onePacket = false) override {
    return write((const uint8_t *)s.data(), s.length(), onePacket);
  }
  void write(const char *s, bool onePacket) {
    return write((const uint8_t *)s, strlen(s), onePacket);
  }
  void write(const char *buf, int len, bool onePacket = false) {
    return write((const uint8_t *)buf, len, onePacket);
  }
  void write(const uint8_t *buf, int len, bool onePacket = false) {
    if (len > 0) { 
      check();
      if (client.connected()) { 
        string s = ReliableStreamFraming::encode(buf, len, sendSequence);
        if (s.empty())
          return; // Escaped payload exceeds the protocol's 65,535-byte limit.
        sendSequence++;
        int n = clientWrite(client, (const uint8_t *)s.data(), s.length(),
            onePacket, 0);
        client.flush();
        if (n <= 0) {
          //Serial.printf("ReliableStream: write error, closing\n");
          resetClient();
        }
        if (n > 0) 
          lastSend = millis();
        //Serial.printf("SEND >>>> %s\n", s.c_str());
      } 
    }             
  }
  int available() { return client.available() || !readBuffer.empty(); }
  size_t getReadBufferHighWaterMark() const {
    return readBuffer.highWaterMark();
  }
  size_t getReadBufferClearThreshold() const {
    return readBuffer.clearThreshold();
  }
  uint32_t getReadBufferOverrunCount() const {
    return readBuffer.overrunCount();
  }
  string read() { 
    string s;
    read(s);
    return s;
  }
  bool writeBusy() { return client.writeBusy(); }
  uint32_t resetCount() const override { return clientResetCount; }
  uint32_t getOnePacketMissCount() const override {
    return clientOnePacketMissCount(client, 0);
  }
  uint32_t getFrameLossCount() const override { return frameLossCount; }
  uint32_t getSequenceDiscontinuityCount() const override {
    return sequenceDiscontinuityCount;
  }
  int read(uint8_t *buf, int len) { 
    string s;
    int packetBytes = read(s);
    if (packetBytes > len)
      return -packetBytes;
    if (packetBytes > 0)
      memcpy(buf, s.data(), packetBytes);
    return packetBytes;
  }
  int read(char *buf, int len) { return read((uint8_t *)buf, len); }
  int read(string &s) {
    s = "";
    check();
    while (client.available()) {
      uint8_t buf[512];
      int n = client.read(buf, sizeof(buf));
      if (n <= 0) {
        Serial.printf("ReliableStream: read error, closing\n");
        resetClient();
        break;
      }
      readBuffer.append(buf, n);
      lastRecv = millis();
    }
    uint8_t sequence = 0;
    if (ReliableStreamFraming::extract(readBuffer, s, sequence)) {
      if (haveReceiveSequence) {
        uint8_t advance = (uint8_t)(sequence - receiveSequence);
        if (advance == 0 || advance > 128)
          sequenceDiscontinuityCount++;
        else
          frameLossCount += advance - 1;
      }
      receiveSequence = sequence;
      haveReceiveSequence = true;
    }
    return s.length(); 
  }
protected:
  template<class U>
  static auto clientWrite(U &c, const uint8_t *buf, size_t len,
      bool onePacket, int) -> decltype(c.write(buf, len, onePacket)) {
    return c.write(buf, len, onePacket);
  }
  template<class U>
  static int clientWrite(U &c, const uint8_t *buf, size_t len,
      bool, long) {
    return c.write(buf, len);
  }
  template<class U>
  static auto clientOnePacketMissCount(const U &c, int)
      -> decltype(c.getOnePacketMissCount()) {
    return c.getOnePacketMissCount();
  }
  template<class U>
  static uint32_t clientOnePacketMissCount(const U &, long) { return 0; }

  void check() { 
    if (!client.connected() && millis() - lastRecon > 1000) {
      reconnect();
      lastRecon = lastSend = lastRecv = millis();
    }
    client.setTimeout(6000);
    if (client.available()) {
      lastRecv = millis();
      inboundTimeoutMs = initialInboundTimeoutMs;
    }
    if (millis() - lastSend > 3000) { 
      lastSend = millis();
      string s = HEARTBEAT_pkt;
      client.write((uint8_t *)s.c_str(), s.length());    
    }
    if (millis() - lastRecv > inboundTimeoutMs) {
      lastRecv = millis();
      inboundTimeoutMs = min(maxInboundTimeoutMs, inboundTimeoutMs * 2);
      //Serial.printf("ReliableStream: readTimeout, closing\n");
      if (client.connected())
        resetClient();
    }
  }
  void resetClient() {
    clientResetCount++;
    haveReceiveSequence = false;
    client.stop();
  }
  bool initialized = false;
  virtual void reconnect() = 0;
public:
  T client;
};

class ReliableTcpServer : public ReliableStream<WiFiClient> {
public: 
  ReliableTcpServer() {}
  ReliableTcpServer(uint16_t p,
      size_t upperClearThreshold = ReliableStream<WiFiClient>::defaultClearThreshold)
      : ReliableStream("", p, upperClearThreshold) {}
  WiFiServer server;
  void reconnect() {
    if (!initialized) {
      server.begin(port);
      initialized = true;
    }
    if (!client.connected()) {
      client = server.available();
      if (client.connected()) { 
        Serial.printf("ReliableTcpServer: connected\n");
      }
    }
  }
};

class ReliableTcpClient : public ReliableStream<WiFiClient> {
public:
  ReliableTcpClient(const char *h, uint16_t p,
      size_t upperClearThreshold = ReliableStream<WiFiClient>::defaultClearThreshold)
      : ReliableStream(h, p, upperClearThreshold) {}
  void reconnect() {
    if (!client.connected()) {
      client.connect(host.c_str(), port, 10000);
      if (client.connected()) { 
        Serial.printf("ReliableTcpClient: connected\n");
      }
    }
  }
};

class ESPNowClient {
public:
  // Fixed storage bounds memory; exceeding the configured threshold clears
  // queued bytes and drops the complete callback payload.
  static constexpr size_t byteQueueCapacity = 4096;
  static constexpr size_t defaultClearThreshold = 2048;
private:
  ReliableStreamByteQueue<byteQueueCapacity> buf;
  SemaphoreHandle_t mutex;
  bool initialized = false;
  uint32_t onePacketMissCount = 0;
public:
  ESPNowMux *enMux;
  string prefix;
  ESPNowClient(ESPNowMux *m = &defaultEspNowMux,
      size_t clearThreshold = defaultClearThreshold)
      : buf(clearThreshold) {
    enMux = m;
  }
  int read(uint8_t *out, size_t n) {
    if (!initialized)
      return 0;
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    int r = buf.read(out, n);
    xSemaphoreGive(mutex);
    return r;
  }
  uint32_t lastSend = 0;
  int write(const uint8_t *buf, size_t n) { 
    enMux->send(prefix.c_str() /*prefix*/, buf, n);
    return n;
  }
  int write(const uint8_t *buf, size_t n, bool onePacket) {
    if (onePacket && n + prefix.length() > ESPNowMux::physicalPacketBytes)
      onePacketMissCount++;
    return write(buf, n);
  }
  uint32_t getOnePacketMissCount() const { return onePacketMissCount; }
  void setClearThreshold(size_t clearThreshold) {
    buf.setClearThreshold(clearThreshold);
  }
  size_t getClearThreshold() {
    return buf.clearThreshold();
  }
  size_t getQueueHighWaterMark() {
    if (!initialized)
      return 0;
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    size_t highWaterMark = buf.highWaterMark();
    xSemaphoreGive(mutex);
    return highWaterMark;
  }
  uint32_t getQueueOverrunCount() {
    if (!initialized)
      return 0;
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    uint32_t overrunCount = buf.overrunCount();
    xSemaphoreGive(mutex);
    return overrunCount;
  }
  bool connected() { return initialized; } 
  void connect() {
    if(!initialized) { 
      mutex = xSemaphoreCreateMutex();
      enMux->registerReadCallback(prefix.c_str() /*prefix*/, 
        [this](const uint8_t *mac, const uint8_t *data, int len){ 
          this->onRecv(mac, data, len); });
      initialized = true;
    }
  }
  int available() {
    if (!initialized)
      return 0;
    // If the take times out, this still accesses buf without owning the mutex.
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    int n = buf.size();
    xSemaphoreGive(mutex);
    return n;
  }
  void flush() {}
  void stop() {  enMux->stop(); }
  void setTimeout(int) {}
  bool writeBusy() { return enMux->writeBusy(); }
  void onRecv(const uint8_t * mac, const uint8_t *in, int len) {
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    buf.append(in, len);
    //Serial.printf("onRecv() %02x:%02x:%02x:%02x:%02x:%02x %d %d\n", 
    //  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len, buf.length());
    xSemaphoreGive(mutex);
  }
};


class ReliableStreamESPNow : public ReliableStream<ESPNowClient> {
public: 
  ReliableStreamESPNow(const char *prefix, bool alwaysBroadcast = false,
      size_t lowerClearThreshold = ESPNowClient::defaultClearThreshold,
      size_t upperClearThreshold = ReliableStream<ESPNowClient>::defaultClearThreshold)
      : ReliableStream(prefix, 0, upperClearThreshold) {
    client.setClearThreshold(lowerClearThreshold);
    client.enMux->alwaysBroadcast = alwaysBroadcast;
  }
  size_t getTransportQueueHighWaterMark() {
    return client.getQueueHighWaterMark();
  }
  uint32_t getTransportQueueOverrunCount() {
    return client.getQueueOverrunCount();
  }
  void reconnect() { client.prefix = host; client.connect(); }
};
#endif // RELIABLESTREAM_H
#endif // ESP32
       //
