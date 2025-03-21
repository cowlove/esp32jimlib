//#include "jimlib.h" todo- remove all static defs from jimlib.h

#ifndef CSIM
#include <esp_task_wdt.h>
#else
//#include "ESP32sim_ubuntu.h" // sigh remove all static defs here too
typedef struct { int timeout_ms, idle_core_mask, trigger_panic; } esp_task_wdt_config_t;  
void esp_task_wdt_init(const esp_task_wdt_config_t *);
void esp_task_wdt_deinit();

#endif

void wdtInit(int sec) {
	esp_task_wdt_config_t c; 
	c.timeout_ms = (sec)*1000; 
	c.idle_core_mask = 0x1; 
	c.trigger_panic = true; 
	esp_task_wdt_deinit(); 
	esp_task_wdt_init(&c);  // include jimlib.h last or this will cause compile errors in other headers
}
