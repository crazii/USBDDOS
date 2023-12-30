# USBDDOS
USB driver stack for DOS

USBDDOS was originally named RWDDOS before released to public, which is only a driver for RetroWave OPL3, and later more drviers added (mouse, keyboard, disk) and renamed to USBDDOS.  

USBDDOS is tested working on following PCs/VMs:  
OCHI: VirtualBox, NEC versa s260(p3)  
UHCI: QEMU, Toshiba portege M200(p4), Compaq Evo N600c(p3), Toshiba Satellite 2410(p4)  

There might be bugs for other PCs, test & feedback is appreiated.   

The Borland++3.1 build (USBDDOS.EXE) can run without a DPMI host, it will enter protected mode directly or with a EMM manager. ~~It's not compatible with many games.  
To play games with a USB mouse, USBDDOSP.EXE is recommended, which also need a DIMI host (i.e. HDPMI.exe).~~ Latest tests with 1.0fix2 shows that USBDDOS is more stable than USBDDOSP.  Like [Bret Johnson's driver](https://bretjohnson.us/), USBDDOSP/USBDDOS need CuteMouse to work for USB mouse. 

# Tips
USBDDOS uses DPMI to save conventional memory for other DOS programs, it uses almost 0 conventional memory (USBDDOS uses ~8K, USBDDOSP uses almost 0). USBDDOSP doesn't need HDPMI32i unless you want Retrowave support, normal HDPMI32 is recommended, unless you want it cope with SBEMU.

If you get problems with the mouse/keyboard driver, make sure those settings in your BIOS settings:
* Disable USB Legacy Support (USB Keyboard/Mouse Support). (some p4 laptop have buggy mouse support in BIOS)
* Disable OnBoard LAN

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
* Or use BC IDE to perform build alternatively
  > cd usbdos\make\
  > bc
* The BC build uses a custom (pseudo) DPMI that limit memmory to: 64K code + 64K data.

DJGPP (native DOS or emu i.e.VirtualBox, cross toolchain on linux not tested):
* ~~Install LFN support, gcc headers are not 8.3 restricted. More info can be found here: https://sta.c64.org/lfnemu.html~~ LFN is not needed anymore because is was needed by C++ headers, and C++ code is changed to C only.
* `make -f usbddos.mak`. the makefile is generated from RHIDE so it make have problem on dependencies.
* Or use RHIDE to perform build
  > cd usbddos\make\
  > rhide
* The DJGPP build depends on a DPMI host, i.e. its CWSDPMI. but HDPMI is recommended.

OpenWatcom:  
Experimental and not supported anymore due to its inline assembly limits. There's an out-dated makefile.wc if you want to make any debugging/improvements.  
If without RetroWave OPL support, OpenWatcom can still work.

# How to Debug
Modify the makefile, or changes compiler arguments in the IDE, add `-DDEBUG=1` and remove `-DNDEBUG`, then rebuild. There will be more debug message shown. The protected mode debugging utilities are not used.

# Requirements
HIMEM.SYS or other XMS manager; EMM386 4.46+(optional, for RetroWave driver); 80386 CPU or later; 

# Features
The code is orginally written as DOS driver for RetroWave OPL3, currently only a simple CDC driver, UHCI tested on a real hardware and OHCI tested on VirtualBox & real harware. But it could be more. the following can be supported:
* - [x] OHCI driver
* - [x] UHCI driver
* - [x] EHCI driver
* - [ ] xHCI driver
* - [x] Hub
* - [x] Keyborad & mouse driver (HID). use `USBDDOSP /hid` to enable
* - [x] Mass storage deivce driver (MSD). use `USBDDOSP /disk` to enable
* - [x] Simple CDC(ACM) driver and RetroWave OPL3. use `USBDDOSP /RW` to enable
* - [ ] Audio and MIDI

# Tested games with Retrowave DOS driver
* Skyroads
* Prince of Persia1 & 2
* Heroes of Jinyong (Jinyong Qun Xia Zhuan), DOS4GW Miles Sound
* Chinese Paladin (Xian Jian Qi Xia Zhuan)
* Theme Hospital, Miles Sound
* DOOM (DJGPP build only, need a modified HDPMI with port trapping feature, see https://github.com/crazii/HX)
* The Jungle Book (DJGPP build only)
* Warcraft II (DJGPP build only)
