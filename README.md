# USBDDOS
USB driver stack for DOS

# Credits
* RetroWaveLib from Sudomaker (folder renamed to RetroWav as 8.3 file name, and tiny code changes, for old compiler) https://github.com/SudoMaker/RetroWave
* This code is somehow based on an old code, usb-driver-under-dos: https://code.google.com/archive/p/usb-driver-under-dos/, with critital changes:
  * Multiple compiler support and code refactoring and major bug fixes.
  * DPMI support and wrapper added
  * IRQ handling added（polling changed to interrupt）
  
# How to Build
Borland C++3.1 (native DOS or emu):
* Setup build env: install the BC toolchains and set path to it. 
* In folder ./Make, execute `make -f makefile.bc`. the makefile is generated from BC utils and doesn't have clean.
* Or use BC IDE to perform build alternatively.
* The BC build uses a custom (pseudo) DPMI that limit memmory to: 64K code + 64K data.

DJGPP (native DOS or emu i.e.VirtualBox, cross toolchain on linux not tested):
* ~~Install LFN support, gcc headers are not 8.3 restricted. More info can be found here: https://sta.c64.org/lfnemu.html~~
* `make -f rwddos.mak`. the makefile is generated from RHIDE so it make have problem on dependencies/clean.
* Or use RHIDE to perform build.
* The DJGPP build depends on a DPMI host, i.e. its CWSDPMI. but HDPMI is recommended.

OpenWatcom:  
Experimental and not supported anymore due to its inline assembly limits. There's an out-dated makefile.wc if you want to make any debugging/improvements.  

# How to Debug
Modify the makefile, or changes compiler arguments in the IDE, add `-DDEBUG=1` and remove `-DNDEBUG`, then rebuild. There will be more debug message shown. The protected mode debugging utilities are not used.

# Requirements
HIMEM.SYS; EMM386 4.46+(for RetroWave driver); 80386 CPU or later; 

# Features
The code is orginally written as DOS driver for RetroWave OPL3, currently only a simple CDC driver, UHCI tested on a real hardware and OHCI tested on VirtualBox & real harware. But it could be more. the following can be supported:
* - [x] OHCI driver
* - [x] UHCI driver
* - [ ] EHCI driver
* - [ ] Keyborad & mouse driver (HID)
* - [x] MSD (mass storage deivce)
* - [x] Simple CDC(ACM) driver and RetroWave OPL3
* - [ ] Audio and midi stuff.

# Tested games with Retrowave DOS driver
* Skyroads
* Prince of Persia1 & 2
* Heroes of Jinyong (Jinyong Qun Xia Zhuan), DOS4GW Miles Sound
* Chinese Paladin (Xian Jian Qi Xia Zhuan)
* Theme Hospital, Miles Sound
* DOOM (DJGPP build only, need a modified HDPMI with port trapping feature, see https://github.com/crazii/HX)
* The Jungle Book
