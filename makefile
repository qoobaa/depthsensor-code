PREFIX=/usr/bin
PART=attiny26
PORT=/dev/ttyS0
PROG=ponyser
CC=$(PREFIX)/avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=$(PART)
OBJ2HEX=$(PREFIX)/avr-objcopy
AVRDUDE=$(PREFIX)/avrdude
AVRDUDEFLAGS=-p $(PART) -P $(PORT) -c $(PROG)
TARGET=sensor

program : $(TARGET).hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$(TARGET).hex
	
%.obj : %.o
	$(CC) $(CFLAGS) $< -o $@
%.hex : %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@
#clean :
#	rm -f *.hex *.obj *.o