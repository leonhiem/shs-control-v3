## General Flags
MCU = attiny13
AVRDIR = /home/leon/avr/avr8-gnu-toolchain-linux_x86_64
CC = $(AVRDIR)/bin/avr-gcc
OC = $(AVRDIR)/bin/avr-objcopy
OD = $(AVRDIR)/bin/avr-objdump
SZ = $(AVRDIR)/bin/avr-size

INCLUDEDIR = $(AVRDIR)/include
LIBDIR = $(AVRDIR)/lib

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)


## Compile options common for all C compilation units.
CFLAGS = $(COMMON) -I$(INCLUDEDIR) -L$(LIBDIR)

#CFLAGS += -Wall -gdwarf-2 -std=gnu99 -DF_CPU=1200000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums 
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -DF_CPU=9600000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums 
CFLAGS += -MD -MP -MT $(*F).o -MF $(@F).d
#CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)

