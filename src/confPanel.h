#include <string>
#include <vector>

using namespace std;
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include "WiFiServer.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_private/wifi.h>


// trim from start
inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

vector<string> split(const char *line, const char *delim) {
  char *buf = strdup(line); 
  std::vector<string> rval;
  for(char *w = strtok(buf, delim); w != NULL; w = strtok(NULL, delim)) {
    string ws(w);
    trim(ws);
    rval.push_back(ws);
  }
  free(buf);
  return rval;
}


/////////////////////////////////////
// CLIENT CODE
class ConfPanelTransportEmbedded;
class ConfPanelParam; 
class ConfPanelClient {
  public:
  int index;
  bool schemaRequested = false;
  vector <ConfPanelParam *> params;
  ConfPanelClient(int i, ConfPanelTransportEmbedded *);
  inline void addFloat(float *ptr, const char *l, float i = 1, const char *f = "%.1f",  
    float mi = 0, float ma = 0, bool w = false);
  inline void addInt(int *ptr, const char *l, float i = 1, const char *f = "%.0f",  
    float mi = 0, float ma = 0, bool w = false);
  inline void addEnum(int *ptr, const char *l, const char *en, bool w = false);
  int attach(ConfPanelParam *p) { 
    params.push_back(p);
    return params.size() - 1;
  } 
  inline string schema();
  inline string values();
  inline void onRecv(const string &);
  inline string readData(); 
};

class ConfPanelParam {
  float inc, min, max, lastValue;
  float *fptr;
  int *iptr;
  bool wrap;
  string label, fmt, enumlist;
  int index;
  ConfPanelClient *owner;
  public:
  ConfPanelParam(ConfPanelClient *o, float *fp, int *ip, const char *l, const char *f, float i = 1, float mi = 0, float ma = 0, bool w = false, const char *el = "none") 
    : owner(o), fptr(fp), iptr(ip), inc(i), min(mi), max(ma), wrap(w) {
      enumlist = el;
      label = l;
      fmt = f; 
      index = owner->attach(this);
      lastValue = get();
  }
  string schemaString() { 
    char buf[256];
    snprintf(buf, sizeof(buf), "%s, %s, %f, %f, %f, %f, %d, %s", 
      label.c_str(), fmt.c_str(), inc, min, max, get(), (int)wrap, enumlist.c_str());
    return string(buf);
  }
  string valueString() { 
    char buf[256];
    snprintf(buf, sizeof(buf), "VALUE %d %d %f", owner->index, index, get());
    return string(buf);
  }
  string readData() { 
    if (get() != lastValue) { 
      lastValue = get();
      return valueString();
    } 
    return "";
  }
  float get() { 
    if (fptr != NULL) 
      return *fptr;
    else
      return *iptr;
  }
  void set(float f) {
    lastValue = f;
    if (fptr != NULL) 
      *fptr = f;
    else
      *iptr = f;
  }

};

inline string ConfPanelClient::schema() {
  char buf[32];
  snprintf(buf, sizeof(buf), "SCHEMA %d %d\n", index, params.size()); 
  string rval = buf;
  for(auto i = params.begin(); i != params.end(); i++) { 
    rval += (*i)->schemaString() + "\n";
  }
  return rval + "END\n";
}
  
inline string ConfPanelClient::values() { 
  string rval;
  for(auto i = params.begin(); i != params.end(); i++) { 
    rval += (*i)->valueString() + "\n";
  }
  return rval;
}

inline string ConfPanelClient::readData() { 
  string rval;
  if (schemaRequested) {
    schemaRequested = false;
    rval += schema();
  }
  for(auto i = params.begin(); i != params.end(); i++) { 
    string s = (*i)->readData();
    if (s.length() > 0)
      rval += s + "\n";
  }
  //Serial.printf("client %d read %s\n", index, rval.c_str());
  return rval;
}

inline void ConfPanelClient::onRecv(const string &buf) { 
  vector<string> lines = split(buf.c_str(), "\n");
  for(string s : lines) {
    int i1, i2;
    float v;
    //Serial.printf("client %d recv %s\n", index, s.c_str());
    if (sscanf(s.c_str(), "SET %d %d %f", &i1, &i2, &v) == 3 && i1 == index && i2 >= 0 && i2 < params.size()) {
      params[i2]->set(v); 
    }
    if (strcmp(s.c_str(), "SCHEMA") == 0) {
      schemaRequested = true; 
    }
  }
}

#if 0 
class ConfPanelParamFloat : public ConfPanelParam { 
public:
  ConfPanelParamFloat(ConfPanelClient *o, float *ptr, const char *l, float i = 1,const char *f = "%.1f", float mi = 0, float ma = 0, bool w = false) 
    : ConfPanelParam(o, ptr, NULL, l, f, i, mi, ma, w) {}
};
class ConfPanelParamEnum : public ConfPanelParam { 
public:
  ConfPanelParamEnum(ConfPanelClient *o, int *ptr, const char *l, const char *en, bool w = false) 
    : ConfPanelParam(o, NULL, ptr, l, "none", 1, 0, 0, w, en) {}
};
#endif

inline void ConfPanelClient::addFloat(float *ptr, const char *l, float i,
    const char *f, float mi, float ma, bool w) {
    new ConfPanelParam(this, ptr, NULL, l, f, i, mi, ma, w);
}
inline void ConfPanelClient::addInt(int *ptr, const char *l, float i,
    const char *f, float mi, float ma, bool w) {
    new ConfPanelParam(this, NULL, ptr, l, f, i, mi, ma, w);
}

inline void ConfPanelClient::addEnum(int *ptr, const char *l, const char *en, bool w) {
    new ConfPanelParam(this, NULL, ptr, l, "none", 1, 0, 0, w, en);
}

class ReliableStreamInterface {
public:
  virtual int read(string &) = 0;
  virtual void write(const string &) = 0;
  void begin(const char *h, uint16_t p) { host = h; port = p; }
protected:
  string host;
  uint16_t port; 
};

template<class T>
class ReliableStream : public ReliableStreamInterface { 
  uint32_t lastSend = 0, lastRecv = 0, lastRecon = 0;
public:
  ReliableStream() {}
  ReliableStream(const char *s, uint16_t p) { begin(s, p); }
  void write(const string &s) {
    if (s.length() > 0) { 
      check();
      if (client.connected()) { 
        int n = client.write((uint8_t *)s.c_str(), s.length()); 
        client.flush();
        if (n <= 0) {
          client.stop();
          Serial.printf("ReliableStream: write error, closing\n");
        }
        if (n > 0) 
          lastSend = millis();
        //Serial.printf("SEND >>>> %s\n", s.c_str());
      } 
    }             
  }
  string read() { 
    string s;
    read(s);
    return s;
  }
  int read(string &s) {
    char buf[1024];
    s = "";
    check(); 
    if (client.connected() && client.available()) { 
      int n = client.read((uint8_t *)buf, sizeof(buf));
      s.assign(buf, n);    
      if (n > 0) {
        lastRecv = millis();
        //Serial.printf("RECV <<<< %s\n", s.c_str()); 
      }
      if (n <= 0) {
        Serial.printf("ReliableStream: read error, closing\n");
        client.stop();
      }
    }
    return s.length(); 
  }
protected:
  void check() { 
    if (!client.connected() && lastRecon - millis() > 1000) {
      reconnect();
      lastRecon = lastSend = lastRecv = millis();
    }
    client.setTimeout(6000);
    if (millis() - lastSend > 3000) { 
      lastSend = millis();
      write("ACK\n");    
    }
    if (millis() - lastRecv > 9000) {
      lastRecv = millis();
      Serial.printf("ReliableStream: readTimeout, closing\n");
      if (client.connected()) 
        client.stop();
    }
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

void ESPNowClientOnDataRecv(const uint8_t * mac, const uint8_t *in, int len);

class ESPNowClient {
  esp_now_peer_info_t peerInfo;
  uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  string buf;
  bool initialized = false;
  SemaphoreHandle_t mutex;
public:
  ESPNowClient() { Instance = this; }
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
    //Serial.printf("ESPNowClient::write(%d)\n", n);
    size_t sent = 0;
    while(sent < n) { 
      // throttle espNOW for reliability
      while(millis() - lastSend < 19) delay(1);
      int pl = min((size_t)200, n - sent);
      esp_now_send(broadcastAddress, (uint8_t *)buf + sent, pl);
      sent += pl;
      lastSend = millis();
    } 
    //Serial.printf("ESPNowClient::write() done\n");
    //delay(1);
    return n;
  }
  bool connected() { return initialized; } 
  void connect() {
    if (!initialized) { 
      int chan = WiFi.channel();
      chan = 4;
      mutex = xSemaphoreCreateMutex();
      WiFi.mode(WIFI_STA);
      esp_wifi_start();
      esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
      esp_now_init();
      esp_now_register_recv_cb(ESPNowClientOnDataRecv);
      memcpy(peerInfo.peer_addr, broadcastAddress, 6);
      peerInfo.channel = chan;  
      peerInfo.encrypt = false;
      esp_now_add_peer(&peerInfo);
      initialized = true;
    }
  }
  int available() { return buf.length(); }
  void flush() {}
  void stop() {}
  void setTimeout(int) {}
  static ESPNowClient *Instance;
  uint8_t recvBuf[1024];
  int recvBufBytes = 0;
  void onRecv(const uint8_t * mac, const uint8_t *in, int len) {
    xSemaphoreTake(mutex, 200 * portTICK_PERIOD_MS);
    string s;
    s.assign((const char *)in, len);
    buf += s;
    //Serial.printf("onRecv() %d %d\n", len, buf.length());
    xSemaphoreGive(mutex);
  }
};

ESPNowClient *ESPNowClient::Instance = NULL;
void ESPNowClientOnDataRecv(const uint8_t * mac, const uint8_t *in, int len) { 
   ESPNowClient::Instance->onRecv(mac, in, len);
}

class ReliableStreamESPNow : public ReliableStream<ESPNowClient> {
  void reconnect() { client.connect(); }
};

class ConfPanelTransportEmbedded {
  uint32_t lastBroadcast = 0;
  WiFiUDP udp;
  bool intialized = false;
  ReliableStreamInterface *stream;
public:
  vector <ConfPanelClient *> clients;
  ConfPanelTransportEmbedded(ReliableStreamInterface *s) : stream(s) {}
  void add(ConfPanelClient *p) { 
      clients.push_back(p);
  }
  void run() {
      string s;
      for (auto c : clients) 
          s += c->readData();    
      if (s.length() > 0) {
        stream->write(s);
      }
      while (stream->read(s)) { 
        onRecv(s.c_str(), s.length());    
      }
      if (millis() - lastBroadcast > 2000) { 
        string s = WiFi.localIP().toString().c_str();
        int n = udp.beginPacket("255.255.255.255", 4444);
        n = udp.write((uint8_t *)s.c_str(), s.length());
        udp.endPacket();
        lastBroadcast = millis();
      }
  }

  LineBuffer lb;
  void onRecv(const char *b, int len) { 
    lb.add((char *)b, len, [this](const char *line) {
      for (auto p : clients) { 
        p->onRecv(line);
      }
  }); 

  }
};

inline ConfPanelClient::ConfPanelClient(int i, ConfPanelTransportEmbedded *t) : index(i) { t->add(this); }
