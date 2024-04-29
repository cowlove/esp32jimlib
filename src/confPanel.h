#include <string>
#include <vector>

using namespace std;
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>


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

class ConfPanelParam; 
class ConfPanelClient {
  public:
  int index;
  bool schemaRequested = false;
  vector <ConfPanelParam *> params;
  ConfPanelClient(int i) : index(i) {}
  inline void addFloat(float *ptr, const char *l, float i = 1,const char *f = "%.1f", 
    float mi = 0, float ma = 0, bool w = false);
  inline void addEnum(float *ptr, const char *l, const char *en, bool w = false);
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
  float *ptr;
  bool wrap;
  string label, fmt, enumlist;
  int index;
  ConfPanelClient *owner;
  public:
  ConfPanelParam(ConfPanelClient *o, float *p, const char *l, const char *f, float i = 1, float mi = 0, float ma = 0, bool w = false, const char *el = "none") 
    : owner(o), ptr(p), inc(i), min(mi), max(ma), wrap(w) {
      enumlist = el;
      label = l;
      fmt = f; 
      index = owner->attach(this);
      lastValue = *ptr;
  }
  string schemaString() { 
    char buf[256];
    snprintf(buf, sizeof(buf), "%s, %s, %f, %f, %f, %f, %d, %s", label.c_str(), fmt.c_str(), inc, min, max, *ptr, (int)wrap, enumlist.c_str());
    return string(buf);
  }
  string valueString() { 
    char buf[256];
    snprintf(buf, sizeof(buf), "VALUE %d %d %f", owner->index, index, *ptr);
    return string(buf);
  }
  string readData() { 
    if (*ptr != lastValue) { 
      lastValue = *ptr;
      return valueString();
    } 
    return "";
  }
  void set(float f) {
    lastValue = *ptr = f;
  }

};

inline string ConfPanelClient::schema() {
  char buf[32];
  snprintf(buf, sizeof(buf), "SCHEMA %d\n", index); 
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

class ConfPanelParamFloat : public ConfPanelParam { 
public:
  ConfPanelParamFloat(ConfPanelClient *o, float *ptr, const char *l, float i = 1,const char *f = "%.1f", float mi = 0, float ma = 0, bool w = false) : ConfPanelParam(o, ptr, l, f, i, mi, ma, w) {}
};
class ConfPanelParamEnum : public ConfPanelParam { 
public:
  ConfPanelParamEnum(ConfPanelClient *o, float *ptr, const char *l, const char *en, bool w = false) : ConfPanelParam(o, ptr, l, "none", 1, 0, 0, w, en) {}
};

inline void ConfPanelClient::addFloat(float *ptr, const char *l, float i,
    const char *f, float mi, float ma, bool w) {
    new ConfPanelParamFloat(this, ptr, l, i, f, mi, ma, w);
}

inline void ConfPanelClient::addEnum(float *ptr, const char *l, const char *en, bool w) {
    new ConfPanelParamEnum(this, ptr, l, en, w);
}

void startServer();
class ConfPanelUdpTransport {
    vector <ConfPanelClient *> clients;
    //WiFiUDP udp;
    bool initialized = false;

public:
    AsyncClient *client = NULL;
    void add(ConfPanelClient *p) { 
        clients.push_back(p);
    }
    int ackTimeout = 0;
    void run() { 
        string s;
        if (!initialized) { 
           	startServer();
            initialized = true; 
        }
        if (ackTimeout++ > 5000 && client != NULL && client->connected()) { 

            s = sfmt("ACK %d\n", (int)millis());
            int n = client->write(s.c_str(), s.length()); 
            ackTimeout = 0;             
        }
        for (auto c : clients) 
            s += c->readData();
        if (s.length() > 0) {
          ackTimeout = 0;             
          if (client != NULL && client->connected()) { 
            int n = client->write(s.c_str(), s.length());              
            //Serial.printf("wrote %d bytes\n", n);
            if (n == 0) {
                client->close();
                Serial.printf("write error, closing\n");
            }
          }
          //udp.beginPacket("255.255.255.255", 4444);
          //udp.write((uint8_t *)s2.c_str(), s2.length());
          //udp.endPacket();
        }
        //if (udp.parsePacket() > 0) {
        //    char buf[1024];
        //    int n = udp.read((uint8_t *)buf, sizeof(buf));
        //    onRecv(buf, n);    
        //}   
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

ConfPanelClient cpc(0);
ConfPanelUdpTransport cup;

static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	//Serial.printf("Got %d bytes\n", len);
	cup.onRecv((const char *)data, len);
}

static void handleError(void* arg, AsyncClient* client, uint8_t err) {
	Serial.printf("Got err %s\n", client->errorToString(err));
  client->close();
	cup.client = NULL;
}

static void handleTimeout(void* arg, AsyncClient* client, uint32_t t) {
	Serial.printf("Got timeout %d\n", t);
  client->close();
	cup.client = NULL;
}

static void handleDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("handleDisconnect()\n");
  client->close();
	cup.client = NULL;
}
static void handleNewClient(void* arg, AsyncClient* client) {
	Serial.printf("handleNewClient()\n");

	//client->setRxTimeout(2000);
  //client->setAckTimeout(2000);
  client->onData(&handleData, NULL);
  client->onError(&handleError, NULL);
  client->onDisconnect(&handleDisconnect, NULL);
  client->onTimeout(&handleTimeout, NULL);
	cup.client = client;
}

AsyncServer *server; 
void startServer() {
  server = new AsyncServer(4444);
  server->onClient(&handleNewClient, &server);
  server->begin();
}
