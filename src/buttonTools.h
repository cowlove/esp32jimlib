#pragma once
#include "jimlib.h"

class DigitalDebounce {
	EggTimer timer;
	bool recentlyPressed;
	uint32_t startPress;
	int lastDuration;
	int lastVal;
public:
	int duration;
	int count;
	DigitalDebounce(int ms = 50) : timer(ms), recentlyPressed(false),count(0) {}
	bool checkOneshot(bool button) {
		bool rval = false; 
		if (button == true) {
			if (timer.tick() && lastVal == false) 
				recentlyPressed = false;
			rval = !recentlyPressed;
			if (rval) {
				count++;
				startPress = millis();
			}
			recentlyPressed = true;
			timer.reset();
			duration = max((uint32_t)1, (uint32_t)millis() - startPress);
		} else {
			duration = 0;
			if (timer.tick()) 
				recentlyPressed = false;
		}
		lastVal = button;
		return rval;
	}
	int checkEndPress() {
		int rval = 0; 
		if (duration == 0 && lastDuration > 0) 
			rval = lastDuration;
		lastDuration = duration;
		return rval;
	}
};

#ifdef ESP32
class RotaryEncoder {
public:
	int pin1, pin3;
	DigitalDebounce a,b;
	int limMin, limMax;
	int value;
	bool wrap;
	int count = 0;
	unsigned long lastChange;

#ifndef CSIM
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	void IRAM_ATTR ISR() {	
		portENTER_CRITICAL_ISR(&(this->mux));
		check();
		portEXIT_CRITICAL_ISR(&(this->mux));
	}
	void begin(void (*ISR_callback)(void)) {
		attachInterrupt(digitalPinToInterrupt(pin1), ISR_callback, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pin3), ISR_callback, CHANGE);
	}
#else
	void begin(void *) {};
#endif
	
	RotaryEncoder(int p1, int p3, int debounce = 5) : a(debounce), b(debounce), pin1(p1), pin3(p3){
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin3, INPUT_PULLUP);
		limMin = 001;
		limMax = 360;
		value = limMin;
		wrap = true;
		lastChange = 0;
	}
	void setLimits(int mn, int mx, bool wrap) {
		limMin = mn;
		limMax = mx;
		this->wrap = wrap;
	}
	void change(int delta) {
		if (delta != 0) { 
			value += delta;
			if (value < limMin) value = wrap ? limMax : limMin;
			if (value > limMax) value = wrap ? limMin : limMax;
		}
	}
	void check() {
		unsigned long now = millis();
		//if (now - lastChange > 500)
		//	Serial.printf("\n\n");
		count++;
		int buta = !digitalRead(pin1);
		int butb = !digitalRead(pin3);
		int oa = a.checkOneshot(buta);
		int ob = b.checkOneshot(butb);
		int delta = 0;
		if (oa && !butb) 
			delta = -1;
		if (ob && !buta)
			delta = +1;
		//Serial.printf("%d/%d %d/%d %d\n", buta, oa, butb, ob, delta );
		
		if (delta != 0) {
			if(lastChange > 0 && now - lastChange < 20)
				delta *= 5;
			lastChange = now;
		}
		change(delta);
	}
};
#endif 

class DigitalButton { 
	DigitalDebounce deb;
	bool inverted;
public:
	int pin;
	int count;
	int mode;
	DigitalButton(int p, bool invert = true, int m = INPUT_PULLUP, int debounceMs = 5) : pin(p), mode(m), inverted(invert), deb(debounceMs) {
		//pinMode(pin, mode);
	}
	bool read() { 
		pinMode(pin, mode);
		bool in = digitalRead(pin);
		return inverted ? !in : in;
	}
	bool check() {
		bool in = read();
		bool rval = deb.checkOneshot(in);
		if (rval) count++;
		return rval;
	}
	int duration() {
		check();
		return deb.duration;
	}
	int checkEndPress() {
		check();
		return deb.checkEndPress();
	}
};

template <class T>
class Changed {
	T old;
	bool first, cas;
public:
	Changed(int changeAtStart = false) : first(true), cas(changeAtStart) {}
	bool changed(T n) { 
		bool r = first ? cas : (n != old);
		old = n;
		first = false;
		return r;
	}
};

class LongShortFilter { 	
	int longPressMs, resetMs;
	unsigned long lastEndTime;
public:
	bool wasLong;
	int last, wasDuration;
	int count, wasCount;
	int lastDuration;
	int events;
	bool countedLongPress;
	bool finishingLong = false;
	LongShortFilter(int longMs, int resetTimeMs) : longPressMs(longMs), resetMs(resetTimeMs) {
		last = events = lastDuration = count = wasCount = lastEndTime = 0;
		countedLongPress = false;
	}
	
	bool check(int l) { 
		unsigned long now = millis();
		last = l;
		int rval = false;
		if (last == 0) { 
			if (lastDuration > 0) {  // press just ended, button up 
				count++;
				lastEndTime = now;
				wasDuration = lastDuration;
				lastDuration = 0;
			}
			if (lastEndTime > 0 && now - lastEndTime >= resetMs) { // resetMS after last button press 
				if (!countedLongPress) {
					wasCount = count;
					rval = true;
				}
				lastEndTime = count = 0;
				countedLongPress = false;
			} 
		} else { 
			if (last >= longPressMs && !countedLongPress) { // long press just finished, button still down
				countedLongPress = true;
				lastDuration = wasDuration = last;
				wasCount = ++count;
				rval = true;
			} else {
				lastDuration = last;
			}
		}
		if (rval == true) {
			events++;
			wasLong = wasDuration >= longPressMs;
		}
		return false;
	}
	bool inProgress() { return ((last != 0) || (lastEndTime > 0 && millis() - lastEndTime < resetMs)) && (countedLongPress == false); }
	int inProgressCount() { return count + ((last != 0) ? 1 : 0); }
	Changed<int> eventCount;
	bool newEvent() { 
		return eventCount.changed(events);
	}
};

class DigitalButtonLongShort { 
	public:
	LongShortFilter filter;
	DigitalButton button;
	DigitalButtonLongShort(int p, int l = 1000, int d = 250) : filter(l, d), button(p) {}
	bool newEvent() { return filter.newEvent(); } 
	void run() { 
		filter.check(button.duration());
	}
	bool newEventR() { run(); return newEvent(); }
	int count() { return filter.wasCount; } 
	bool inProgress() { return filter.inProgress(); } 
	int inProgressCount() { return filter.count; } 
	int wasLong() { return filter.wasLong; } 
};

template<class T> 
class StaleData {
	uint64_t timeout, lastUpdate, lastChange;
	T value, invalidValue;
	bool chg = false; 
public:
	StaleData(int t, T i) : lastUpdate(0), timeout(t), invalidValue(i) {} 
	bool isValid() { return millis() - lastUpdate < timeout; }
	operator T&() { return getValue(); }
	StaleData<T>& operator =(const T&v) {
		chg = value != v;
		value = v;
		lastUpdate = millis();
		if (chg) 
			lastChange = millis();
		return *this;
	}
	T& getValue() { return isValid() ? value : invalidValue; }
	bool changed() { 
		bool rval = chg;
		chg = false;
		return rval && isValid(); 
	}
	uint64_t age() { 
		return millis() - lastUpdate;
	}
	uint64_t timeSinceChange() { 
		return millis() - lastChange;
	}
};

template<class T> 
class ChangedData {
	T value;
	bool chg = false;
	bool first = true;
public:
	ChangedData(T i) {} 
	operator T&() { return value; }
	ChangedData<T>& operator =(const T&v) {
		chg = value != v || first;
		value = v;
		first = false;
		return *this;
	}
	bool changed() { 
		bool rval = chg;
		chg = false;
		return rval; }
};


class PinPulse { 
public:
    int pin;
    uint64_t toggleTime = 0;
    PinPulse(int p, int initval = 0) : pin(p) { pinMode(p, OUTPUT); digitalWrite(p, initval); } 
    void  pulse(int v, int ms) { 
        toggleTime = ms > 0 ? millis() + ms: 0;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, v);
    }
    void run() { 
        if (toggleTime > 0 && millis() >= toggleTime) {
            toggleTime = 0;
            pinMode(pin, OUTPUT);
            digitalWrite(pin, !digitalRead(pin));
        }
    }
};

