TARGET := output/usbddosp.exe
all: $(TARGET)

ifdef OSTYPE #defined on linux

CC = i586-pc-msdosdjgpp-gcc
SRC = $(shell find .. -name '*.c')
clean:
	$(RM) $(OBJS)

else #DOS/FreeDOS

CC = gcc
SRC = $(shell dir ..\*.c /b/s/l/-p) #-p will remove /p in DIRCMD, which causes problems if used.
RM := del
clean:
	$(RM) output\*.o
    
endif #OSTYPE

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
EXCLUDES = dpmi_wc.c sample.c dpmi_ldr.c

SRC := $(notdir $(SRC))
SRC := $(filter-out $(EXCLUDES), $(SRC))

OBJS := $(patsubst %.c,output/%.o,$(SRC))

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

output/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
