#ifndef _WDT_H_
#define _WDT_H_

int wdt_enable(int maxPeriodMS, bool isForSleep); 
void wdt_reset(); 
void wdt_disable(); 
int wdt_sleep(int maxPeriodMS);
void wdt_init(void); 

#endif // _WDT_H_
