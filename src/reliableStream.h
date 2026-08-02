#ifdef ESP32
#ifndef RELIABLESTREAM_H
#define RELIABLESTREAM_H
#include <string>
#include "Arduino_CRC32.h" 
#include "reliableStreamFraming.h"
using std::string;

class ReliableStreamInterface {
public:
  virtual int read(string &) = 0;
  void write(const string &s) { write(s, false); }
  virtual void write(const string &, bool onePacket) = 0;
  virtual uint32_t getOnePacketMissCount() const = 0;
  virtual uint32_t resetCount() const = 0;
  void begin(const char *h, uint16_t p) { host = h; port = p; }
protected:
  string host;
  uint16_t port; 
};


// Legacy naming note:
//
// ReliableStream is not a stream protocol in the application sense. It is a
// logical-packet framing layer over a byte-oriented transport. The transport
// may be TCP, UDP, ESP-NOW, or another implementation that coalesces,
// fragments, or otherwise does not preserve write boundaries. A successful
// read returns one complete logical packet or nothing; it does not promise
// delivery, ordering, retransmission, or persistence across transport loss.
//
// The legacy class/file name is retained for API compatibility. A more
// accurate future name would be ReliablePacket. Likewise, the historical
// "-1 ACK" control payload is not an acknowledgement: it does not confirm
// application delivery and is deliberately sent outside application framing,
// so it is never returned to the caller. It is a liveness heartbeat/probe used
// to exercise the underlying channel.
//
// Frames use ~<escaped-byte-count>|<escaped-payload>|<CRC32>. Only the three
// reserved printable bytes '~', '|' and '%' are percent-escaped. Length and
// CRC validation provide atomic reads and an unescaped '~' provides a clean
// synchronization point after loss. Newlines, NULs and other payload bytes are
// not modified by the framing layer.
//
// write(payload, true) is a one-physical-packet performance hint. It never
// rejects a logical write. Packet transports may count misses when the final
// encoded frame requires fragmentation; stream transports ignore the hint.

template<class T>
class ReliableStream : public ReliableStreamInterface { 
  uint32_t lastSend = 0, lastRecv = 0, lastRecon = 0;
  uint32_t clientResetCount = 0;
  static constexpr uint32_t initialInboundTimeoutMs = 3000;
  static constexpr uint32_t maxInboundTimeoutMs = 30000;
  uint32_t inboundTimeoutMs = initialInboundTimeoutMs;
  const string HEARTBEAT_pkt = "-1 ACK\n";
  string readBuffer;
public:
  ReliableStream() {}
  ReliableStream(const char *s, uint16_t p) { begin(s, p); }
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
        string s = ReliableStreamFraming::encode(buf, len);
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
  int available() { return client.available() || readBuffer.length() > 0; }
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
      readBuffer.append((const char *)buf, n);
      lastRecv = millis();
    }
    ReliableStreamFraming::extract(readBuffer, s);
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
  ReliableTcpServer(uint16_t p) : ReliableStream("", p) {}
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
  ReliableTcpClient(const char *h, uint16_t p) : ReliableStream(h, p) {}
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
  string buf;  // TODO change to a CBQ
  SemaphoreHandle_t mutex;
  bool initialized = false;
  uint32_t onePacketMissCount = 0;
public:
  ESPNowMux *enMux;
  string prefix;
  ESPNowClient(ESPNowMux *m = &defaultEspNowMux)  {
    enMux = m;
  }
  int read(uint8_t *out, size_t n) {
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    int r = min(n, buf.length());
    memcpy(out, buf.c_str(), r);
    buf.erase(0, r);
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
  int available() { return buf.length(); }
  void flush() {}
  void stop() {  enMux->stop(); }
  void setTimeout(int) {}
  bool writeBusy() { return enMux->writeBusy(); }
  void onRecv(const uint8_t * mac, const uint8_t *in, int len) {
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    string s;
    s.assign((const char *)in, len);
    buf += s;
    //Serial.printf("onRecv() %02x:%02x:%02x:%02x:%02x:%02x %d %d\n", 
    //  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len, buf.length());
    xSemaphoreGive(mutex);
  }
};


class ReliableStreamESPNow : public ReliableStream<ESPNowClient> {
public: 
  ReliableStreamESPNow(const char *prefix, bool alwaysBroadcast = false) : ReliableStream(prefix, 0) {
    client.enMux->alwaysBroadcast = alwaysBroadcast;
  }
  void reconnect() { client.prefix = host; client.connect(); }
};
#endif // RELIABLESTREAM_H
#endif // ESP32
       //
