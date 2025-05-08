#ifndef RELIABLESTREAM_H
#define RELIABLESTREAM_H
#include <string>
#include "Arduino_CRC32.h" 
using std::string;

class ReliableStreamInterface {
public:
  virtual int read(string &) = 0;
  virtual void write(const string &) = 0;
  void begin(const char *h, uint16_t p) { host = h; port = p; }
protected:
  string host;
  uint16_t port; 
};


// reads potentially unaligned and packetized data from client interface, 
// prepends a CRC prefix and a separator suffix string, and provides a more reliable
// packet interface.  also sends periodic ACK packets (that are deliberately 
// dropped due to bad CRC), and handles reinitalizing the client layer on error
// or timeout 
// serious TODO deficiencies, see notes below 

template<class T>
class ReliableStream : public ReliableStreamInterface { 
  uint32_t lastSend = 0, lastRecv = 0, lastRecon = 0;
  const string ACK_pkt = "-1 ACK\n";
  string readBuffer;
public:
  string separator = string("\nXOXEND\n");
  ReliableStream() {}
  ReliableStream(const char *s, uint16_t p) { begin(s, p); }
  void write(const string &s) { return write(s.c_str(), s.length()); }
  void write(const char *buf, int len) { return write((const uint8_t *)buf, len); }
  void write(const uint8_t *buf, int len) { 
    if (len > 0) { 
      check();
      if (client.connected()) { 
        string s;
        s.assign((char *)buf, len);
        Arduino_CRC32 crc32;
        uint32_t cksum = crc32.calc((uint8_t const *)s.c_str(), s.length());
        s = sfmt("%x ", cksum) + s + separator;
        int n = client.write((uint8_t *)s.c_str(), s.length()); 
        client.flush();
        if (n <= 0) {
          //Serial.printf("ReliableStream: write error, closing\n");
          client.stop();
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
  int read(uint8_t *buf, int len) { 
    // TODO!!! this should not be the main read function, switch over to 
    // the one that returns a string.  This will discard the rest of a packet
    // if it's larger than len 
    check();
    if (readBuffer.length() == 0 && (!client.connected() || !client.available())) 
      return 0;

    while(client.available()) { 
      int n = client.read(buf, len); 
      if (n <= 0) {
        Serial.printf("ReliableStream: read error, closing\n");
        client.stop();
        break; 
      }
      string s;
      s.assign((char *)buf, n);
      lastRecv = millis();
      //printf("RECV <<<< %s\n", s.c_str()); 
      readBuffer += s;
    } 
  
    auto endp = readBuffer.find(separator); 
    if (endp != string::npos) { 
      string s = readBuffer.substr(0, endp);
      //printf("RECV got a packet of length %d, readBuf len %d\n", endp, readBuffer.length());
      uint32_t cksum = 0;
      if (s.find(" ") != string::npos && sscanf(s.c_str(), "%x ", &cksum) == 1) {  
        s = s.substr(s.find(" ") + 1, s.find(separator));
        memcpy(buf, s.c_str(), min((int)len, (int)s.length()));
        //printf("RECV     returning: '%s'\n", s.c_str());
        readBuffer.erase(0, endp + separator.length());
        Arduino_CRC32 crc32;
        uint32_t cksum2 = crc32.calc((uint8_t const *)s.c_str(), s.length());
        if (cksum != cksum2) { 
          if (s.find(ACK_pkt) != string::npos)
            printf("RECV bad cksum, discarding packet\n");
          s = "";
        }
        return s.length(); 
      }
      readBuffer.erase(0, endp);
    }
    return 0;
  }
  int read(char *buf, int len) { return read((uint8_t *)buf, len); }
  int read(string &s) {
    // UGH see note in read() above.  
    char buf[4096];
    s = "";
    int n = read(buf, sizeof(buf)); 
    if (n > 0) 
      s.assign(buf, n);    
    return s.length(); 
  }
protected:
  void check() { 
    if (!client.connected() && lastRecon - millis() > 1000) {
      reconnect();
      lastRecon = lastSend = lastRecv = millis();
    }
    client.setTimeout(6000);
#ifndef CSIM
    if (millis() - lastSend > 3000) { 
      lastSend = millis();
      string s = ACK_pkt + separator;
      client.write((uint8_t *)s.c_str(), s.length());    
    }
    if (millis() - lastRecv > 9000) {
      lastRecv = millis();
      //Serial.printf("ReliableStream: readTimeout, closing\n");
      if (client.connected()) 
        client.stop();
    }
#endif
  } 
  bool initialized = false;
  T client;
  virtual void reconnect() = 0;
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
public:
  string prefix;
  ESPNowClient()  {}
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
    espNowMux.send(prefix.c_str() /*prefix*/, buf, n);
    return n;
  }
  bool connected() { return initialized; } 
  void connect() {
    if(!initialized) { 
      mutex = xSemaphoreCreateMutex();
      espNowMux.registerReadCallback(prefix.c_str() /*prefix*/, 
        [this](const uint8_t *mac, const uint8_t *data, int len){ 
          this->onRecv(mac, data, len); });
      initialized = true;
    }
  }
  int available() { return buf.length(); }
  void flush() {}
  void stop() {  espNowMux.stop(); }
  void setTimeout(int) {}
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
    espNowMux.alwaysBroadcast = alwaysBroadcast;
  }
  void reconnect() { client.prefix = host; client.connect(); }
};
#endif // RELIABLESTREAM_H
