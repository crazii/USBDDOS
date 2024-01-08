# USBDDOS
USB driver stack for DOS

USBDDOS was originally named RWDDOS before released to public, which is only a driver for RetroWave OPL3, and later more drviers added (mouse, keyboard, disk) and renamed to USBDDOS.  

USBDDOS is tested working on following PCs/VMs:  
* OCHI: VirtualBox, NEC versa s260(p3)  
* UHCI: QEMU, Toshiba portege M200(p4), Compaq Evo N600c(p3), Toshiba Satellite 2410(p4)  
* EHCI: VirtualBox, NEC versa s260, Lenovo Thinkpad T540p

There might be bugs for other PCs, test & feedback is appreiated.   

USBDDOS uses DPMI to perform PCI bus master DMA (MMIO), and also save conventional memory for other DOS programs, it uses little conventional memory (USBDDOS uses ~12K, USBDDOSP uses almost 0).

The Borland++3.1/Open Watcom build (USBDDOS.EXE) can run without a DPMI host,  
it has a builtin 16 bit protected mode functions which will enter protected mode directly or with a EMM manager.  
~~It's not compatible with many games.
To play games with a USB mouse, USBDDOSP.EXE is recommended, which also need a DIMI host (i.e. HDPMI.exe).~~ Latest tests with 1.0fix2 shows that USBDDOS is more stable than USBDDOSP.  
Like [Bret Johnson's driver](https://bretjohnson.us/), USBDDOSP/USBDDOS need CuteMouse to work for USB mouse. 

# Tips
USBDDOSP doesn't need HDPMI32i unless you want Retrowave support, normal HDPMI32/CWSDPMI is recommended, unless you want it cope with SBEMU.

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
The build scripts support multiple hosts: Linux, Windows(WSL/MinGW), DOS/FreeDOS.  
Three set of toolchians are supported: DJGPP, Borland C++ 3.1, Open Watcom v2.  
Here's a quick comparison of different toolchains and its generated executables:  
|             | DJGPP         | Borland C++3.1 | Open Watcom |
|-------------|:-------------:|:--------------:|:-----------:|
|Linux Host   |Yes            |No              |Yes          |
|Windows Host |Yes (WSL/MinGW)|No              | Yes         |
|DOS Host     |Yes            |Yes             |Yes          |
|Makefile     |Makefile       | Makefile.BC    | Makefile.WC |
|Debug Build  | ```DEBUG=1``` | ```-DDEBUG```  |```DEBUG=1```|
|Output Name  | usbddosp.exe  | usbddos.exe    | usbddos.exe |
|Executable            ||
|DPMI Host Required    |Yes   |No              |No           |
|Conventional Mem Usage|0K    |12K             |12K          |

## DJGPP setup
* Download DGJPP from here: https://github.com/andrewwutw/build-djgpp  
* Run ```djgpp/setenv``` : ```. /djgpp/setenv``` or ```source /djgpp/setenv```  
* Make: ```make```  

For DOS host, download from here: https://www.delorie.com/djgpp/  and set the envs: ```set PATH=%PATH%;C:\DJGPP```, ```set DJGPP=C:\DJGPP\DJGPP.ENV```

## Borland C++ setup
The makefile uses hard coded path: ```C:\BORLANDC``` for includes and libs.  
* Set exec path: ```set PATH=%PATH%;C:\BORLANDC```  
* Make: ```make -f Makefile.BC```

## Open Watcom setup
* Download Open Watcom from here: https://github.com/open-watcom/open-watcom-v2
* Set env: ```. /watcom/owsetenv.sh``` or ```source /watcom/owsetenv.sh```  
* Make: ```wmake -f Makefile.WC```

DOS setup: ```set PATH=%PATH%;C:\OW2\BINW```, ```set WATCOM=C:\OW2```, 
```set INCLUDE=C:\OW2\H```

# How to Debug
Add ```DEBUG=1``` on make commandline, the built executable will have its logs/assertion enabled.  Use ```-DDEBUG``` for Borland C++'s make file.

# Requirements
* HIMEM.SYS or other XMS manager
* EMM386 4.46+(optional, for RetroWave driver)
* 80386 CPU or later
* MS-DOS (FreeDOS is still in debugging)

# Features
The fllowwing drivers are added/planned:
* - [x] OHCI driver
* - [x] UHCI driver
* - [x] EHCI driver
* - [ ] xHCI driver
* - [x] Hub
* - [x] Keyborad & mouse driver (HID). use `USBDDOSP /hid` to enable
* - [x] Mass storage class driver (MSC). use `USBDDOSP /disk` to enable
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
