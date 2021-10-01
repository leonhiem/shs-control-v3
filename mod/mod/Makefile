#
# Top level Makefile
#


include Make.inc

## General Flags
PROJECT = main
TARGET = main.elf

## Linker flags
LDFLAGS +=  -Wl,-Map=main.map
CFLAGS += -I. 

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Objects that must be built in order to link
OBJECTS = main.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all:	$(TARGET) main.hex main.eep main.lss size

## Compile
main.o: main.c
	@echo `pwd`
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	$(OC) -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-$(OC) $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	$(OD) -h -S $< > $@

size: ${TARGET}
	@echo
#       @avr-size -C --mcu=${MCU} ${TARGET}
	@$(SZ) --target=ihex ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) main.elf dep/* main.hex main.eep main.lss main.map *.o.d

prog:
	avrdude -p t13 -c avrispmkII -P usb -e -U flash:w:main.hex:i

fuses:
	#avrdude -p t13 -c avrispmkII -P usb -U lfuse:w:0x6a:m # CKDIV8=0: 1.2MHz
	avrdude -p t13 -c avrispmkII -P usb -U lfuse:w:0x7a:m # CKDIV8=1: 9.6MHz
	avrdude -p t13 -c avrispmkII -P usb -U hfuse:w:0xff:m



## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)
