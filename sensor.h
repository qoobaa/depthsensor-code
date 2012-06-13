#ifndef _SENSOR_H_
#define _SENSOR_H_

// PORTA configuration
#define LED1 1
#define LED2 4
#define LED3 5
#define PROG 2
#define REL1 6
#define REL2 7

// Pointers to values in EEPROM
#define PMINIMUM (void*)0x00
#define PMAXIMUM (void*)0x02

// Before returning the result read pressure ADCCOUNT times
#define ADCCOUNT 63

void initialize(void);
uint16_t pressure(void);
void wait_for_prog(void);
void calibrate(void);
int main(void);

#endif
