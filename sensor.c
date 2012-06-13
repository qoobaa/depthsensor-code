#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "sensor.h"

// 1 if some errors occured
uint8_t Error;

// Values from EEPROM
uint16_t Minimum, Average, Maximum;

// Current value of pressure; SwitchState of PROG button
volatile uint16_t Current, SwitchState;

ISR(TIMER0_OVF0_vect)
{
  // PROG key debouncing
  SwitchState = (SwitchState << 1) | (PINA & _BV(PROG));
}

ISR(TIMER1_OVF1_vect)
{
  // LED1 blinking
  PORTA ^= _BV(LED1);
  // LED2 and LED3 blinking if error
  if(Error)
    {
      PORTA ^= _BV(LED2) | _BV(LED3);
    }
}

ISR(ADC_vect)
{
  // Read current ADC value
  Current = ADCW;
}

void initialize(void)
{
  // PORTA initial configuration
  // LED1-3 - inactive
  // REL1-2 - inactive
  // PROG - internal pull-up enabled
  PORTA = _BV(LED1) | _BV(LED2) | _BV(LED3) | _BV(REL1) | _BV(REL2) | _BV(PROG);

  // DDRA configuration
  // LED1-3 - outputs
  // REL1-2 - outputs
  DDRA = _BV(LED1) | _BV(LED2) | _BV(LED3) | _BV(REL1) | _BV(REL2);

  // ADC Enable
  // ADC Interrupt Enable
  ADCSR = _BV(ADEN) | _BV(ADIE);

  // ADC0 channel change
  // Internal Voltage Reference with external capacitor at AREF pin
  ADMUX = _BV(REFS1) | _BV(REFS0);

  // Enable sleep mode with ADC noise reduction
  set_sleep_mode(SLEEP_MODE_ADC);

  // Timer/Counter0 setup for key debouncing
  // Clock0 prescaler = CK/256
  TCCR0 = _BV(CS02);
  
  // Timer/Counter1 setup for blinking
  // Clock1 prescaler = CK/2048
  TCCR1B = _BV(CS13) | _BV(CS12);

  // Enable Timer/Counter0,1 overflow interrupts
  TIFR = _BV(TOV0), _BV(TOV1);

  // Enable interrupts
  sei();

  // If PROG button is pressed enter to calibration mode
  if(!(PINA & _BV(PROG))) calibrate();
  
  // Read values stored in EEPROM
  eeprom_busy_wait();
  Minimum = eeprom_read_word(PMINIMUM);
  eeprom_busy_wait();
  Maximum = eeprom_read_word(PMAXIMUM);
  Average = (Maximum + Minimum) / 2;
  
  // Check the configuration
  if(
     (Minimum == 0) ||
     (Maximum == 0) ||
     (Maximum == Minimum) ||
     (Minimum >= 0x0ff) ||
     (Maximum >= 0x0ff) ||
     (Minimum > Maximum)
     )
  {
    // Oops - errors in configuration
    Error = 1;
    TIMSK |= _BV(TOIE1);
    while(1) { }
  }

  // LED1 on - sensor is working now!
  PORTA &= ~_BV(LED1);
}

uint16_t pressure(void)
{
  uint8_t i;
  uint16_t result = 0;
  for(i = 0; i < ADCCOUNT; i++)
    {
      // Start conversion
      ADCSR |= _BV(ADSC);
      // Sleep mode with ADC noise reduction
      sleep_mode();
      // Sum result, skip two LSB to exclude noise
      result += Current >> 2;
    }
  // Return average
  return (result / ADCCOUNT);
}

void wait_for_prog(void)
{
  // Start debouncing
  TIMSK |= _BV(TOIE0);
  // wait for falling edge
  while(SwitchState != 0xf000) { }
  SwitchState = 0;
  // Stop debouncing
  TIMSK &= ~_BV(TOIE0);
}

void calibrate(void)
{
  uint16_t min, max;
  // Timer/Counter0,1 Overflow Interrupt Enable - LED1 blinking
  TIMSK |= _BV(TOIE1);

  // Calibrate minimum value
  PORTA &= ~_BV(LED2);
  wait_for_prog();
  min = pressure();
  PORTA |= _BV(LED2);

  // Calibrate maximum value
  PORTA &= ~_BV(LED3);
  wait_for_prog();
  max = pressure();
  PORTA |= _BV(LED3) | _BV(LED1);

  // Save values in EEPROM
  eeprom_busy_wait();
  eeprom_write_word(PMINIMUM, min);

  eeprom_busy_wait();
  eeprom_write_word(PMAXIMUM, max);
  
  // Timer/Counter0,1 Overflow Interrupt Disable
  TIMSK &= ~_BV(TOIE1);
}

int main(void)
{
  // Do the initialization
  initialize();

  uint16_t p;
  while(1)
    {
      p = pressure();
      if(p < Minimum)
	{
	  // Minimum value reached
	  PORTA &= ~_BV(LED2) & ~_BV(REL1);
	  // Pump water into until Avg level is reached
	  while(pressure() < Average) { }
	  PORTA |= _BV(LED2) | _BV(REL1);
	}
      else if(p > Maximum)
	{
	  // Maximum value reached
	  PORTA &= ~_BV(LED3) & ~_BV(REL2);
	  // Pump out water until Avg level is reached
	  while(pressure() > Average) { }
	  PORTA |= _BV(LED3) | _BV(REL2);
	}
    }
  // NEVER REACHED
}
