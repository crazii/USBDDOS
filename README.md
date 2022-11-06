# USBDDOS
USB driver stack for DOS

# Credits
* RetroWaveLib from Sudomaker (folder renamed to RetroWave as 8.3 file name, and tiny code changes, for old compiler) https://github.com/SudoMaker/RetroWave
* This code is somehow based on an old code, usb-driver-under-dos: https://code.google.com/archive/p/usb-driver-under-dos/, with critital changes:
  * Multiple compiler support and code refactoring
  * DPMI wrapper added
  * IRQ handling added
  
# How to Build
Borland C++3.1 (native DOS or emu):
* Setup build env: install the BC toolchains and set path to it. 
* In folder ./Make, execute `make -f makefile.bc`. the makefile is generated from BC utils and doesn't have clean.
* Use BC idle to perform build.
* The BC build uses a custom (pseudo) DPMI that limit memmory to: 64K code + 64K data.

DJGPP (native DOS or emu, cross toolchain on linux not tested):
* Install LFN support, gcc headers are not 8.3 restricted. more info can be found here: https://sta.c64.org/lfnemu.html
* `make -f rwddos.mak`. the makefile is generated from RHIDE so it make have problem on dependencies/clean.
* Or use RHIDE to perform build.
* The DJGPP build depend on a DPMI host, i.e. its CWSDPMI. but HDPMI is recommended.

OpenWatcom:  
Experimental and not supported anymore due to its inline assembly limits. There's an out-dated makefile.wc if you want to make any debugging/improvements.  

# How to Debug
Modify the makefile, or changes compiler arguments, add `-DDEBUG=1` and remove `-DNDEBUG`, then rebuild. There will be more debug message shown. The protected mode debugging utilities are not used.

# Requirements
HIMEM.SYS; EMM386 4.46+(for RetroWave driver); 80386 CPU or later; 

# Features
The code is orginally written as DOS driver for RetroWave OPL3, currently only a simple CDC driver, UHCI tested on a real hardware and OHCI tested on VirtualBox. But it could be more. the following can be added:
* EHCI driver
* MSD (mass storage deivce)
* Audio and midi stuff.
