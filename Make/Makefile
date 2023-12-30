TARGET := output/usbddosp.exe
all: $(TARGET)

ifdef PROMPT # dirty hack to detect DOS, won't work if Linux has PROMPT defined
CC = gcc
SRC = $(shell dir ..\*.c /b/s)
RM := del
clean:
	$(RM) output\*.o
else #PROMPT
CC = i586-pc-msdosdjgpp-gcc
SRC = $(shell find .. -name '*.c')
clean:
	$(RM) $(OBJS)
endif #PROMPT

DEBUG ?= 0

CFLAGS = -x c -std=gnu11 -march=i386 -Os -pedantic -Wstrict-aliasing -fno-exceptions -pedantic-errors \
	-Wreturn-type -Wunused -Wuninitialized -Wundef -Wcast-align -Wwrite-strings -Wconversion -Wsign-compare -Werror \
	-I../ \

LDFLAGS = -lm

ifeq ($(DEBUG),0)
LDFLAGS += -s
CFLAGS += -DNDEBUG
else
CFLAGS += -DDEBUG=1
endif

VPATH = .. ../USBDDOS ../USBDDOS/CLASS ../USBDDOS/DPMI ../USBDDOS/HCD ../RetroWav ../RetroWav/Platform ../RetroWav/Protocol ../RetroWav/Board
EXCLUDES = MasterGear.c MiniBlaster.c Linux_SPI.c POSIX_SerialPort.c STM32_HAL_SPI.c Win32_SerialPort.c dpmi_wc.c sample.c dpmi_ldr.c

SRC := $(notdir $(SRC))
SRC := $(filter-out $(EXCLUDES), $(SRC))

OBJS := $(patsubst %.c,output/%.o,$(SRC))

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

output/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
