TARGET := output/usbddosp.exe
all: $(TARGET)

ifdef PROMPT # dirty hack to detect DOS, won't work if Linux has PROMPT defined
CC = gcc
SRC = $(shell dir ..\*.c /b/s)
RM := del
clean:
	$(RM) output\*.o
else #PROMPT
ifeq (, $(shell which i586-pc-msdosdjgpp-gcc)) # If i586-pc-msdosdjgpp-gcc is not in the path
ifeq (, $(shell which gcc)) # If gcc is also not in the path
$(error "No compiler found in $(PATH), make sure that either i586-pc-msdosdjgpp-gcc or gcc (DJGPP) is installed.")
else # gcc IS in the path
CC = gcc
endif # If gcc is also not in the path
else # i586-pc-msdosdjgpp-gcc IS in the path
CC = i586-pc-msdosdjgpp-gcc
endif # If i586-pc-msdosdjgpp-gcc is not in the path
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
CFLAGS += -DNDEBUG -DDEBUG=0
else
CFLAGS += -DDEBUG=1
endif

VPATH = .. ../USBDDOS ../USBDDOS/CLASS ../USBDDOS/DPMI ../USBDDOS/HCD ../RetroWav ../RetroWav/Platform ../RetroWav/Protocol ../RetroWav/Board
EXCLUDES = MasterGear.c MiniBlaster.c Linux_SPI.c POSIX_SerialPort.c STM32_HAL_SPI.c Win32_SerialPort.c dpmi_wc.c sample.c

SRC := $(notdir $(SRC))
SRC := $(filter-out $(EXCLUDES), $(SRC))

OBJS := $(patsubst %.c,output/%.o,$(SRC))

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

output/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
