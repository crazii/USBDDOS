TARGET := output/usbddosp.exe
all: $(TARGET)

DOS_ENV =
ifdef PROMPT
DOS_ENV = MSDOS
endif
ifdef OS_NAME
DOS_ENV = FREEDOS
endif

ifeq "$(DOS_ENV)" ""

CC = i586-pc-msdosdjgpp-gcc
SRC = $(shell find .. -name '*.c')
BUILD ?= $(shell git log -n1 --format=format:"%h"), DJGPP

clean:
	$(RM) $(OBJS)

else #DOS/FreeDOS

CC = gcc
SRC = $(shell dir ..\*.c /b/s/l/-p) #-p will remove /p in DIRCMD, which causes problems if used.
BUILD = DJGPP DOS
RM := del
clean:
	$(RM) output\*.o
    
endif

DEBUG ?= 0

CFLAGS = -std=gnu11 -march=i386 -Os -pedantic -Wstrict-aliasing -fno-exceptions -pedantic-errors \
	-Wreturn-type -Wunused -Wuninitialized -Wundef -Wcast-align -Wwrite-strings -Wconversion -Wsign-compare -Werror \
	-I../ \
	-DUSBDDOS_BUILD="\"$(BUILD)\"" \

LDFLAGS = -lm

ifeq ($(DEBUG),0)
LDFLAGS += -s
CFLAGS += -DNDEBUG
else
CFLAGS += -DDEBUG=1
endif

VPATH = .. ../USBDDOS ../USBDDOS/CLASS ../USBDDOS/DPMI ../USBDDOS/HCD ../USBDDOS/DPMI/djgpp \
	../RetroWav ../RetroWav/Platform ../RetroWav/Protocol ../RetroWav/Board

EXCLUDES = dpmi_wc.c sample.c dpmi_ldr.c

SRC := $(notdir $(SRC))
SRC := $(filter-out $(EXCLUDES), $(SRC))

OBJS := $(patsubst %.c,output/%.o,$(SRC))

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

#don't use strict warning on this file
output/gormcb.o: gormcb.c
	$(CC) -march=i386 -Os -fno-exceptions -c $< -o $@

output/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
