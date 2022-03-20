#ifndef TTGO_TS_H
#define TTGO_TS_H
#include "jimlib.h"

// mutex to serialize SD card and TFT writes. 
Mutex mutexSPI;

void open_TTGOTS_SD() { 
	for (int retries = 0; retries < 2; retries++) { 	
		ScopedMutex lock(mutexSPI);
		Serial.print("Initializing SD card...");
		//SPI.begin(15, 2, 14);
		if (SD.begin(13, 15, 2, 14)) {
			Serial.println("initialization done.");
			return;
		}
		Serial.println("initialization failed!");
		delay(100);
	}
	Serial.println("giving up");
}

#ifndef UBUNTU

void printDirectory(File dir, int numTabs) {
  while(true) {
     File entry =  dir.openNextFile();
     if (! entry) {
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');   // we'll have a nice indentation
     }
     // Print the name
     Serial.print(entry.name());
     /* Recurse for directories, otherwise print the file size */
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       /* files have sizes, directories do not */
       Serial.print("\t\t");
       Serial.println(entry.size());
     }
     entry.close();
	}
}

void printSD() { 
	File root = SD.open("/");
	if (root) {
		printDirectory(root, 0);
		root.close();
	}
}
#else
void printSD() {}
#endif
#endif


#ifdef UBUNTU
#include <fcntl.h>
#include <unistd.h>
#endif

#ifdef ESP32


static bool logFileBusySPI = false;
template <class T>
void SDCardBufferedLogThread(void *p);

template <class T>
class SDCardBufferedLog {
	int writeBlockSz;
	T *writeBuf;
public:
	int dropped = 0;
	int maxWaiting = 0;
	int written = 0;
#ifndef UBUNTU
	QueueHandle_t queue;
	int timo;
	int flushInterval;
	bool exitNow = false;
	const char *filename;

	void exit() { 
		exitNow = true;
		Serial.printf("Exiting, qsize %d, written %d, free heap %d\n", (int)uxQueueMessagesWaiting(queue), 
			written, ESP.getFreeHeap());
		T v;
		add(&v);
		while(exitNow) {
			yield();
			delay(1);
		};
	}
	bool textMode;
public:
	String currentFile;
	StaticQueue_t qb;
	SDCardBufferedLog(const char *fname, int len, int timeout, int flushInt, bool textMode = true, int wb = 32) : 
		filename(fname), 
		flushInterval(flushInt), 
		writeBlockSz(wb),
		timo((timeout+portTICK_PERIOD_MS-1)/portTICK_PERIOD_MS) {
			
		queue = xQueueCreate(len, sizeof(T));
		writeBuf = new T[writeBlockSz];
		this->textMode = textMode;
		xTaskCreate(SDCardBufferedLogThread<T>, "SDCardBufferedLogThread", 8192, (void *)this, tskIDLE_PRIORITY, NULL );   
	}
	~SDCardBufferedLog() { 
		 this->exit();
		 vQueueDelete(queue);
		 printSD();
		 SD.end();
		 logFileBusySPI = false;
		 delete writeBuf;
	}
	void add(const T *v) {
		add(v, timo);
	}
	void add(const T &v) {
		add(&v, timo);
	}
	void add(const T *v, int t) {
		if (xQueueSend(queue, v, (t+portTICK_PERIOD_MS-1)/portTICK_PERIOD_MS) != pdTRUE) 
			dropped++;
		int waiting = uxQueueMessagesWaiting(queue);
		logFileBusySPI = waiting > 20;
		if (waiting > maxWaiting)
			maxWaiting = waiting;
			
	}
	int queueLen() {
		return uxQueueMessagesWaiting(queue);
	}
	void setFilename() { 
		int fileVer = 1;
		char fname[20];
		File f;
		if (strchr(filename, '%')) { // if filename contains '%', scan for existing files  
			for(fileVer = 1; fileVer < 999; fileVer++) {
				snprintf(fname, sizeof(fname), filename, fileVer);
				ScopedMutex lock(mutexSPI);
				if (!(f = SD.open(fname, FILE_READ))) {
					f.close();
					break;
				}
			} 
		}
		snprintf(fname, sizeof(fname), filename, fileVer);
		currentFile = String(fname);
	}

	void *thread() {
		uint64_t lastWrite, startTime, lastFlush;
		lastFlush = lastWrite = startTime = millis();
		File f;

		open_TTGOTS_SD();
		setFilename();
		{
			ScopedMutex lock(mutexSPI);
			SD.remove((char *)currentFile.c_str());
			f = SD.open((char *)currentFile.c_str(), FILE_WRITE);
		}
		Serial.printf("Opened %s\n", currentFile.c_str());
		int idx = 0;
		while(!exitNow) { // TODO: doesn't flush queue before exiting 	
			if (xQueueReceive(queue, writeBuf + idx, timo) == pdTRUE) {
				if (!exitNow) {
					uint64_t now = millis();
					if (f) { 
						if (textMode == true) { 
							ScopedMutex lock(mutexSPI);
							f.println((*writeBuf).toString());
						} else { 
							idx = (idx + 1) % writeBlockSz;
							if (idx == 0) {
								ScopedMutex lock(mutexSPI);
								size_t s = f.write((uint8_t *)writeBuf, sizeof(T) * writeBlockSz);
								if (s > 0) written += writeBlockSz;
							}
						}
					}
					lastWrite = now;
					//Serial.printf("WROTE\n");
				}
			}
			uint64_t now = millis();
			if (now - lastFlush >= flushInterval) {
				if (f) {
					ScopedMutex lock(mutexSPI);
					f.flush();
				}
				lastFlush = now;
			}
			int waiting = uxQueueMessagesWaiting(queue);
			logFileBusySPI = waiting > writeBlockSz;
		}
		if (f) {
			ScopedMutex lock(mutexSPI);
			f.close();
		}
		exitNow = false;
		Serial.printf("task out\n");
		vTaskDelete(NULL);
	}
	void flush() { 
		while(uxQueueMessagesWaiting(queue) > 0) 
			sleep(1);
	}
#else
public:
	String currentFile;
	int fd = -1;
	void add(const T *v, int timo = 0) {
		if (fd != -1)  int s = write(fd, (void *)v, sizeof(T));
		//printf("%s LOG\n", v->toString().c_str()); 
	}
	void add(const T &v, int timo = 0) { 
		add(&v);
	}
	SDCardBufferedLog(const char *fname, int len, int timeout, int flushInt, bool textMode = true) {
		currentFile = fname;
		char buf[64];
		snprintf(buf, sizeof(buf), fname, 1);
		if (buf != string("-") && buf != string("+"))
			fd = open(buf, O_WRONLY | O_CREAT, 0666);
	}
	~SDCardBufferedLog() { close(fd); }
	int queueLen() { return 0; }
	void flush();
#endif
};	

template <class T>
void SDCardBufferedLogThread(void *p) {
	((SDCardBufferedLog<T> *)p)->thread(); 
}

class ChangeTimer { 
	public:
	float lastVal = 0;
	uint64_t lastChangeMillis;
	ChangeTimer() : lastChangeMillis(0) {}
	float unchanged(float v) { 
		if (v == lastVal) { 
			float rval = (millis() - lastChangeMillis) / 1000.0;
			return rval;
		} else { 
			lastVal = v;
			lastChangeMillis = millis();
			return 0.0;
		}
	}
};

// JDisplay jd;
// JDisplayItem<float> f1(&jd,10,10,"LABEL1:", "%+02.2f");
// JDisplayItem<int> f2(&jd,10,20,"LABEL2:", "%02d");

// jd.begin();
// f2 = 0;
// f1 = 1.1;

#ifndef UBUNTU
#include <Adafruit_GFX.h>               // Core graphics library
#include <Adafruit_ST7735.h>            // Hardware-specific library

#define TFT_CS 16
#define TFT_RST 9  
#define TFT_DC 17
#define TFT_SCLK 5   
#define TFT_MOSI 23  
#define ST7735
#endif

void JDisplayUpdateThread(void *);
class JDisplayItemBase;
class JDisplay {
	std::vector<JDisplayItemBase *> items;
	Semaphore changeSem;
public:
	int textSize, xoffset, yoffset, lastUpdatedItemIndex = 0;
	bool autoupdate;
	static JDisplay *instanceP;
	JDisplay(int tsize = 1, int x = 0, int y = 0, bool au = false) : textSize(tsize), xoffset(x), yoffset(y), autoupdate(au) {
		instanceP = this;
	}
	static const struct Colors { 
		int lf, lb, vf, vb; 
	} defaultColors; 
#ifndef UBUNTU 
	Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
	void begin() { 
		{
			ScopedMutex lock(mutexSPI);
			pinMode(27,OUTPUT); 		//Backlight:27  TODO:JIM 27 appears to be an external pin 
			digitalWrite(27,HIGH);		//New version added to backlight control
			tft.initR(INITR_18GREENTAB);                             // 1.44 v2.1
			tft.fillScreen(ST7735_BLACK);                            // CLEAR
			tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);           // GREEN
			tft.setRotation(1);
			tft.setTextSize(textSize); 
		}
		forceUpdate();
		xTaskCreate(JDisplayUpdateThread, "JDisplayUpdateThread", 8192, this, tskIDLE_PRIORITY, NULL);
	}
	void clear() {
		ScopedMutex lock(mutexSPI);
		tft.fillScreen(ST7735_BLACK);                            // CLEAR
	}
	void printAt(int x, int y, const char *s, int fg, int bg, int size) { 
		ScopedMutex lock(mutexSPI);
		x = x + xoffset;
		y = y + yoffset;
		tft.setTextColor(fg, bg); 
		tft.setCursor(x, y);
		tft.setTextSize(size); 
		tft.print(s);
	}
	void setRotation(int n) { tft.setRotation(n); }
#else
	void setRotation(int) {}
	static const int xsize = 40, ysize = 20;
	char lines[ysize][xsize];
	void begin() {
		for(int y = 0; y < ysize; y++)  {
			for(int x = 0; x < xsize; x++) { 
				lines[y][x] = ' ';
			}
			lines[y][xsize - 1] = '\0';
		}
	}
	void clear() {}
	static bool displayToConsole;
	void printAt(int x, int y, const char *f, int, int, int) {
		if (displayToConsole) {
			char *line = lines[y/10];
			int flen = strlen(f);
			for (int n = 0; n < flen; n++) { 
				line[x/6 + n] = f[n];
			}
			::printf("%s\n", line);
		}
	}
#endif
	void markChange() { changeSem.give(); } 
	void waitChange(int tmo = portMAX_DELAY) { changeSem.take(tmo); }

	void addItem(JDisplayItemBase *i) { 
		items.push_back(i);
	}
	inline void forceUpdate() { update(true, false); }
	inline bool update(bool update, bool onlyOne);	
};

JDisplay *JDisplay::instanceP = NULL;

class JDisplayEditor;
class JDisplayItemBase {
friend JDisplayEditor;
	int x, y, updates;
	int lb, lf, vb, vf;
	JDisplay *d;
	String val, lastVal;
	bool labelInverse = false, valueInverse = false;
	bool first = true;
	int labelFontSize = 0;
public:
	bool changed = false;
	int labelSpace = 0;
	std::string label;
	JDisplay::Colors color;
	JDisplayItemBase(JDisplay *d, int x, int y, const char *label, JDisplay::Colors colors, int labelFontSize) {
		this->labelFontSize = labelFontSize;
		this->x = x;
		this->y = y;
		this->label = label;
		this->d = d;
		this->color = colors;
		updates = 0;
		if (d != NULL)
			d->addItem(this);
	}
	void setInverse(bool li, bool vi) {
		if (li != labelInverse || vi != valueInverse) {
			labelInverse = li;
			valueInverse = vi;
			changed = true;
			if (d != NULL) 
				d->markChange();
		}
		//update(changed);
	}
	bool update(bool force) {
		if (d == NULL)
			return false;
		if (first || ++updates % 1000 == 1) 
			force = true;
		if (changed) { 
			force = true;
			changed = false; 
		}
		first = false;
#ifdef UBUNTU
		labelFontSize = d->textSize = 1;
#endif
		if (force)
			d->printAt(x * d->textSize, y * d->textSize, label.c_str(), labelInverse ? color.lb : color.lf , labelInverse ? color.lf : color.lb, 
			labelFontSize != 0 ? labelFontSize : d->textSize);
		if (force || val != lastVal) {
			int uc = 0; // count the unchanged characters
			while(!force && uc < val.length() && uc < val.length() && val[uc] == lastVal[uc] )
				uc++;
			int xpixel = labelSpace 
				+ (x + uc * 6) * d->textSize 
				+ 6 * strlen(label.c_str()) * (labelFontSize != 0 ? labelFontSize : d->textSize);
			d->printAt(xpixel, y * d->textSize, val.c_str() + uc, valueInverse ? color.vb : color.vf, valueInverse ? color.vf : color.vb, d->textSize);
			lastVal = val;
			return true;
		}
		return false;
	}
	void setValueString(const String &s) {
		val = s;
		if (lastVal != val && d != NULL) {
			d->markChange();
			if (d->autoupdate)
				update(true);
		}
	}
};

inline bool JDisplay::update(bool force, bool onlyOne) { 
	for (std::vector<JDisplayItemBase *>::iterator it = items.begin() + lastUpdatedItemIndex; it != items.end(); it++) { 
		if ((*it)->update(force)) {
			if (onlyOne) {
				lastUpdatedItemIndex = it - items.begin();
				return true;
			}
		}
	}
	lastUpdatedItemIndex = 0;
	return false;
}

template<class T>
class JDisplayItem : public JDisplayItemBase { 
	const char *fmt;
public:
	T value;
	JDisplayItem(JDisplay *d, int x, int y, const char *label, const char *format, int labelFontSize = 0,  JDisplay::Colors colors = JDisplay::defaultColors) :
		JDisplayItemBase(d, x, y, label, colors, labelFontSize), fmt(format) {}
	JDisplayItem<T>& operator =(const T&v) { setValue(v); return *this; }

	std::function<String(const T&)> toString = [this](const T& v)->String { 
		char buf[64];
		snprintf(buf, sizeof(buf), this->fmt, v);
		return String(buf);
	};		
	void setValue(const T&v) { 
		value = v;
		setValueString(toString(v));
	}
};


#ifndef UBUNTU
const JDisplay::Colors JDisplay::defaultColors = { ST7735_GREEN, ST7735_BLACK, ST7735_WHITE, ST7735_BLACK };
#else // UBUNTU
const JDisplay::Colors JDisplay::defaultColors = { 0, 0, 0, 0 };
bool JDisplay::displayToConsole = false;

class ESP32sim_jdisplay : public ESP32sim_Module {
	IntervalTimer sec = IntervalTimer(1.0);
	void parseArg(char **&a, char **la) override {
		if (strcmp(*a, "--jdisplay") == 0) JDisplay::displayToConsole = true;
	}	
	void setup() override {}
	void loop() override { 
		if (sec.tick(millis() / 1000.0) && JDisplay::instanceP != NULL) JDisplay::instanceP->forceUpdate();
	}
	void done() override {}
} esp32sim_jdisplay;
#endif // else UBUNTU

class JDisplayEditableItem;

class JDisplayEditor {
	std::vector<JDisplayEditableItem *> items;
	bool editing;
	int selectedItem;
public:
	RotaryEncoder re;
	JDisplayEditor(int p1, int p2) : re(p1, p2) {}
	void add(JDisplayEditableItem *i) { 
		items.push_back(i);
	}
	void begin() {
		re.limMin = 0;
		re.limMax = items.size() - 1;
		editing = false;
		re.wrap = false;
		re.value = 0;
		selectedItem = 0;
		sortItems();
		//re.begin( [this]()->void{ this->re.ISR(); });
	}
	inline void negateSelectedValue(); 
	inline void update(); 
	inline void buttonPress(bool longpress);			
	inline void sortItems();
};

class JDisplayEditableItem : public JDisplayItem<float> { 
protected:
	typedef JDisplayItem<float> Parent;
public:
	bool recentChange = false;
	bool changed() { 
		if (recentChange) {
			recentChange = false;
			return true;
		}
		return false;
	}
	float value, newValue, increment;
	float min, max;
	bool wrap;
	float *attached = NULL;
	enum { UNSELECTED, SELECTED, EDITING } state;
	JDisplayEditableItem(JDisplay *d, int x, int y, const char *label, const char *format, 
		JDisplayEditor *ded, float inc = 1, float *att = NULL, float mi = -100000, float ma = +10000, bool wr = false) : 
		increment(inc), min(mi), max(ma), wrap(wr), attached(att), Parent(d, x, y, label, format) {
			if (ded != NULL) { 
				ded->add(this);
			}
		}
	void update(bool force = false) {
		if (state == EDITING) {
			Parent::setValue(newValue);
			Parent::setInverse(false, true);
		} else { 
			if (attached != NULL && value != *attached) { 
				value = *attached;
				Parent::changed = true;
			}
			Parent::setValue(value);
			Parent::setInverse(state == SELECTED, false);
		}
		//Parent::update(force);
	};
	void setValue(float v) { 
		value = v;
		Parent::setValue(v);
		if (attached != NULL) {
			*attached = value;
		}
		update();
	}
	void attach(float *f) { attached = f; value = *attached; }
};

inline void JDisplayEditor::negateSelectedValue() { 
	if (!editing) { 
		items[selectedItem]->value *= -1.0;
	} else {
		items[selectedItem]->newValue *= -1.0;
	}
	items[selectedItem]->update();
}
inline void JDisplayEditor::sortItems() { 
	std::sort(std::begin(items), std::end(items), 
	[](JDisplayEditableItem * a, JDisplayEditableItem *b) {
		return a->x != b->x ? a->x < b->x : a->y < b->y;  
	});
}

inline void JDisplayEditor::update() { 
	if (!editing) { 
		if (selectedItem != re.value) { 
			selectedItem = re.value;
			for(int n = 0; n < items.size(); n++) {
				items[n]->state = (n == selectedItem) ? JDisplayEditableItem::SELECTED : 
					JDisplayEditableItem::UNSELECTED;
				//items[n]->update();
			}
		}
	} else { 
		items[selectedItem]->newValue = items[selectedItem]->value + re.value * items[selectedItem]->increment;
		items[selectedItem]->update();
	}
	for(int n = 0; n < items.size(); n++) 
		items[n]->update();
}


			
inline void JDisplayEditor::buttonPress(bool longpress) { 
	if (!editing) { 
		editing = true;
		selectedItem = re.value;
		JDisplayEditableItem *it = items[selectedItem];
		it->state = JDisplayEditableItem::EDITING;
		it->newValue = it->value;
		it->update();
		re.limMin = (it->min - it->value) / it->increment;
		re.limMax = (it->max - it->value) / it->increment;
		re.wrap = it->wrap;
		re.value = 0;
	} else { 
		editing = false;
		if (longpress == false) { 
			if (items[selectedItem]->newValue != items[selectedItem]->value) 
				items[selectedItem]->recentChange = true;
			items[selectedItem]->setValue(items[selectedItem]->newValue);
		}
		items[selectedItem]->state = JDisplayEditableItem::SELECTED;
		re.limMin = 0;
		re.limMax = items.size() - 1;
		re.wrap = false;
		re.value = selectedItem;
	}
	items[selectedItem]->update(true);
}

void JDisplayUpdateThread(void *p) { 
	JDisplay *jd = (JDisplay *)p;
	jd->forceUpdate();
#ifndef UBUNTU	
	esp_task_wdt_delete(NULL);	
	while(true) {
		jd->waitChange(10); 
		esp_task_wdt_reset();
		while(logFileBusySPI == false) {
			esp_task_wdt_reset();
			if (jd->update(false, true) == false || logFileBusySPI)
				break;
		}
		delayMicroseconds(10);
	}
#endif
}

#endif