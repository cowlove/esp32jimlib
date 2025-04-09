#pragma once
#include "jimlib.h"

class FakeMutex {
	public:
	void lock() {}
	void unlock() {}
};

class FakeSemaphore { 
 public:
	FakeSemaphore(int max = 1, int init = 1) {}
	bool take(int delay = 0) { return true; } 
	void give() {}
};

class Mutex {
	SemaphoreHandle_t xSem;
 public:
	Mutex() { 
		xSem = xSemaphoreCreateCounting(1,1);
		unlock();
	}
	void lock() { xSemaphoreTake(xSem, portMAX_DELAY); } 
	void unlock() { xSemaphoreGive(xSem); }
};

class Semaphore { 
	SemaphoreHandle_t xSem;
 public:
	Semaphore(int max = 1, int init = 1) { 
		xSem = xSemaphoreCreateCounting(max, init);
	}
	bool take(int delay = portMAX_DELAY) { return xSemaphoreTake(xSem, delay); } 
	void give() { xSemaphoreGive(xSem); }
	int getCount() { return uxSemaphoreGetCount(xSem); } 
};

class ScopedMutex {
	Mutex *mutex; 
  public:
	ScopedMutex(Mutex &m) : mutex(&m) { mutex->lock(); } 
	ScopedMutex(FakeMutex &m) : mutex(NULL) {} 
	~ScopedMutex() { if (mutex != NULL) mutex->unlock(); } 
};

template <class T>  
class CircularBoundedQueue { 
	Semaphore empty, full;
	int size, head, tail;
	T *array;
public:
	CircularBoundedQueue(int s) : size(s), empty(s, s), full(s, 0) {
		array = new T[size];
		head = tail = 0;
	}
	T *peekHead(int tmo) {
		if (!empty.take(tmo)) 
			return NULL;

		T *rval = &array[head];
		head = (head + 1) % size;
		return rval;
	}
	void postHead() {
		full.give();
	}
	T *peekTail(int tmo) {
		if (!full.take(tmo)) 
			return NULL;

		T *rval = &array[tail];
		tail = (tail + 1) % size;
		return rval;
	}
	void freeTail() {
		empty.give();
	}
	int getCount() { return full.getCount(); } 
};
