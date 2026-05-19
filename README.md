# Netrunner01/USBDDOS — fork

## Status

**Current version: `v1.0.0-alpha.2` (May 15, 2026)** — housekeeping
release; no new code or features.

Every patch from `v1.0.0-alpha.1` was merged into upstream
`crazii/USBDDOS` between May 13 and May 15, 2026. Upstream master is
at `b1308fb`. The alpha.2 binaries on the release page are built
directly from upstream master and are byte-identical (Watcom) or
near-identical (DJGPP, differing only in an embedded build-hash) to
the alpha.1 binaries. If alpha.1 is already running on your system,
there is no reason to update; alpha.2 exists for build-provenance
clarity, not as a functional change.

**The "alpha" label still means real-hardware verification is
pending — NOT that the code is unstable or broken.** Specifically:

* All upstream code builds clean with `-Werror` on Open Watcom v2 and
  DJGPP gcc 12.2.0.
* The 7-test QEMU regression suite passes 7/7
  (`ohci-init`, `ohci-kbd`, `ohci-msc`, `uhci-init`, `ehci-init`,
  `ohci-hub-hid`, `ohci-audio`).
* Every patch was derived from authoritative documentation: the
  USB 2.0 spec (for the Gap 4 hub-class fix), Linux `pci-quirks.c`
  and `ohci-hcd.c` (for Gaps 2, 3, 6), Apple Darwin `IOUSBFamily`
  (for the pending Gap 7), the NEC µPD720101 / ALi M5237 / SiS 630 /
  OPTi 82C861 chip datasheets, and Benjamin Lunt's FYSOS Book 8
  (for Gap 8).
* What's still missing: a regression pass against the actual silicon
  each chip-quirk fix targets. QEMU implements *spec-compliant* USB
  devices; the patched paths exist precisely because real silicon
  deviates from spec.

**Promotion to beta or v1.0.0** requires at least one independent
real-hardware test report per patched gap. If you have NEC µPD720101,
ALi M5237 / M1543C, SiS 7001, OPTi 82C861, or a Mac-Mini-2011-class
Intel PCH platform and can run a smoke test, [file an
issue](https://github.com/Netrunner01/USBDDOS/issues) with the results
either way — that's the single most useful contribution this project
needs right now.

## Why this fork exists

When this fork was started in May 2026, upstream `crazii/USBDDOS` had
not received a maintainer commit since February 2024 and the issue
tracker had accumulated bug reports with no resolutions. The fork's
purpose was to provide a public home for a focused set of fixes that
users had identified but never seen merged.

**That situation has since changed.** Between May 13 and May 15, 2026,
upstream resumed activity and merged every fix in this fork's
`v1.0.0-alpha.1` patch series (PRs #22–#30 against `crazii/USBDDOS`).
The fork's code-side mission is therefore complete — the patches are
now upstream, where they belong.

**The fork continues to exist as a binary-hosting layer.** Many
real-hardware users of legacy DOS USB don't compile their own
drivers, and upstream doesn't publish pre-built binaries. This fork
publishes binary releases (built from upstream master, reproducible
by anyone with Open Watcom v2 or DJGPP) as a convenience for testers
and end users. It also remains a staging area for fixes that haven't
yet been written or upstreamed (see "What this fork does NOT do" for
the current known-unimplemented list).

If upstream begins publishing its own binary releases, this fork can
sunset to read-only.

## Who this fork is for

You probably want this if you've hit one of these and there isn't a
working solution upstream:

* **System hangs on USBDDOS load** on a 386/486/Pentium-era board with
  an **ALi M5237** OHCI controller, or an OEM **ALi M1543C
  southbridge**. (Fixed: Gap 6.)
* **System hangs on USBDDOS load** during BIOS USB-Legacy SMM handoff
  on motherboards with stuck SMI ownership — common on **Mac Mini
  2011**, **Sandy Bridge ThinkPads**, **Intel Cougar Point chipsets**,
  some VIA southbridges. (Partially addressed: Gap 3; full EHCI
  handoff still pending.)
* **USB hub does not work** — devices behind a hub fail to enumerate
  even though USBDDOS sees the hub itself. (Fixed: Gap 4.)
* **Driver works for keyboard but crashes when an unsupported USB
  device is plugged in** (audio, printer, vendor-specific). Cleanup
  asserts at `hcd.c(91)`. (Fixed: HCD teardown commit.)
* **OHCI driver dies silently on certain controllers** after an
  Unrecoverable Error interrupt — the recovery handler was empty.
  (Fixed: Gap 1.)
* **PS/2 keyboard ghosting / spurious interrupts** while USBDDOS is
  loaded on a NEC µPD720101 or SiS 7001 OHCI controller because BIOS
  left Legacy Support active. (Fixed: Gap 5.)
* **Random init failures on OPTi 82C861 (FireLink)** or older SiS
  OHCI silicon where `HcFmInterval` doesn't latch on first write.
  (Fixed: Gap 2.)
* **Headless emulator testing or serial-console real-hardware
  debugging** needs `_LOG` output on COM1, which upstream gates off.
  (Fixed: Debug commit.)
* **Cross-building from Linux** with the DJGPP or Open Watcom v2
  toolchain hits "file not found" on `#include` paths because of
  mixed-case originals. (Fixed: Build commit.)

If your problem isn't on this list, this fork is unlikely to help —
it does NOT add new features beyond what upstream offers (USB
1.1/2.0 host controllers, HID keyboard/mouse, mass storage, CDC,
hub class). Two known bugs in upstream are deliberately NOT yet
patched here: NEC `kErrataNECIncompleteWrite` write-1-to-clear
retries on `HcRhPortStatus` (Gap 7 in our planning notes) and full
EHCI USBLEGSUP/USBLEGCTLSTS BIOS handoff for the Intel ICH4+
systems. Both are tracked.

## Background

A research fork of [crazii/USBDDOS](https://github.com/crazii/USBDDOS)
maintained at upstream commit `54345d0` (Feb 2024). The patch series
in this fork was developed against the **Rescue-D2** project, a
1.44 MB FreeDOS rescue floppy targeting 386–486 / early-Pentium
hardware (NEC, ALi, SiS, and OPTi OHCI silicon). The Rescue-D2 use
case puts more breadth-pressure on USBDDOS's chip-quirk handling
than the typical RetroWave-OPL3 / retro-gaming use cases upstream
was tuned for, which surfaced the gaps below.

## What this fork adds

Ten discrete commits on top of upstream `54345d0`. Each is a single
logical change with a detailed commit message and inline source
comments explaining the mechanism, hardware exposure, and spec
references; `git log 54345d0..master` gives the full series.

| Topic | Brief | Related upstream issue |
|---|---|---|
| Build | Case-correct `#include` paths for cross-build on Linux | [#17](https://github.com/crazii/USBDDOS/issues/17) |
| Debug | Enable COM1 logging in `_LOG` path for headless emulator testing | [#19](https://github.com/crazii/USBDDOS/issues/19) |
| Gap 1 | OHCI UnrecoverableError recovery per OHCI 1.0a §5.3.1.1 | — |
| Gap 6 | Skip HcFmInterval access on ALi M5237 to prevent system lockup | — |
| Gap 5 | Disable OHCI Legacy Support emulation after BIOS handoff | — |
| Gap 3 | Bound the SMM-handoff wait loop (~1s cap) | — |
| Gap 2 | INITRESET-equivalent verify-and-retry for HcFmInterval/HcPeriodicStart latching | — |
| Gap 8 | Wait POTPGT × 2ms after SetPortPower | — |
| Gap 4 | Fix invalid `ClearPortFeature(PORT_RESET)` hub-port-reset spec violation | — |
| HCD | Relax `HCD_RemoveDevice` teardown asserts to logged best-effort cleanup | [#7](https://github.com/crazii/USBDDOS/issues/7) |

The Build commit duplicates work `stanwebber` already did in
[stanwebber/USBDDOS](https://github.com/stanwebber/USBDDOS) (8 commits
dated April 25, 2025, byte-identical fix shape). Both were derived
independently from the same observation; credit for the prior
independent work goes to stanwebber.

## What this fork does NOT do

* **Not a drop-in replacement for upstream.** It does not address
  every reported issue in the upstream tracker, and behavior outside
  the changed code paths is unaffected.
* **Real-hardware verification is pending for all six OHCI gap
  fixes.** Each gap was identified against authoritative documentation
  (OHCI 1.0a spec, Linux `pci-quirks.c`, NEC µPD720101 user manual,
  SiS 630 datasheet, FYSOS Book 8) or silicon-vendor errata captured
  in Apple Darwin's `IOUSBFamily`. The quirk paths the fixes target
  are NOT exercised by QEMU's spec-compliant `pci-ohci` emulation. So
  while the patches build cleanly with `-Werror` on both toolchains
  and don't regress the QEMU-side 5/5 baseline regression suite, none
  of them is yet confirmed to behave correctly on the real silicon
  the fix is meant for.
* **One known OHCI gap remains unpatched:**
  * Gap 7 — NEC `kErrataNECIncompleteWrite` write-1-to-clear retries
    on `HcRhPortStatus`. Fix sketched from Apple Darwin
    `IOUSBFamily/AppleUSBOHCI_RootHub.cpp`, not yet written. Affects
    µPD720101 silicon under specific load patterns.
* **EHCI USBLEGSUP/USBLEGCTLSTS BIOS handoff is not implemented.**
  Gap 3 handles the OHCI side, but the EHCI-side analogue (which
  Linux's `quirk_usb_disable_ehci` covers, including the
  `EHCI_USBLEGCTLSTS_SOOE` SMI-on-Ownership-Change-Enable bit) is
  the most likely root cause of unresolved upstream hangs on Intel
  Cougar Point / Panther Point platforms — see crazii/USBDDOS#10 and
  the Gigabyte P67-DS3-B3 thread on VOGONS.
* **xHCI support is not implemented.** Upstream lists it as a planned
  feature; this fork does not extend the planned-feature scope.
* **No upstream PR campaign.** Upstream `crazii/USBDDOS` is treated
  as functionally dormant. Engagement, where it happens, is per-issue:
  deferring case-fix attribution to `stanwebber` on
  [#17](https://github.com/crazii/USBDDOS/issues/17), and offering the
  COM1 log change as an answer on
  [#19](https://github.com/crazii/USBDDOS/issues/19).

## Building

Build instructions are unchanged from upstream — see the [How to
Build](#how-to-build) section below. The fork builds clean with
`-Werror` on:

* Open Watcom v2 (May 2026 snapshot or later) — produces the
  deployment binary `usbddos.exe`.
* DJGPP cross gcc 12.2.0 — produces `usbddosp.exe`.

The Watcom build is bit-stable across rebuilds at the same commit.
The DJGPP build embeds a build-ID string (the git short-hash), so the
binary's bytes differ when the same source is rebuilt from a different
commit even though the code is identical.

## Reporting issues

This fork's tracker is on this repository
([issues page](https://github.com/Netrunner01/USBDDOS/issues)). Issues
that are about upstream USBDDOS rather than this fork's specific
changes belong on [upstream's tracker](https://github.com/crazii/USBDDOS/issues).

## Common symptoms this fork addresses

If you arrived here from a web search, the strings below are the
actual error output or symptom descriptions from the bugs this fork
patches. They are listed verbatim so search engines return this
README as a hit for users hunting the symptoms in their own logs.

* `assertion failed: result` at `../USBDDOS/HCD/hcd.c(91)` — fixed by
  the HCD-teardown commit, in combination with Gap 4 for the hub
  enumeration path.
* `assertion failed: result` at `../USBDDOS/HCD/hcd.c(93)` or
  `(95)` — same root cause, same fix.
* `HUB port N resetting` followed by `HUB port N reset 103` repeated
  ~4 times, then `USB Removing HCD device` and assertion — Gap 4
  (invalid `ClearPortFeature(PORT_RESET)`).
* USBDDOS hangs at load on an ALi M1543C / ULi southbridge or a
  standalone ALi/ULi M5237 OHCI controller with no log output — Gap 6
  (`HcFmInterval` access lockup).
* USBDDOS hangs at load on a system with BIOS USB Legacy Support
  enabled that won't release SMM ownership — Gap 3 (unbounded SMM
  handoff loop). This is partial coverage; full coverage of Intel
  PCH-class platforms also requires the EHCI USBLEGSUP handoff
  which is not yet implemented.
* PS/2-emulated keyboard ghosts characters or fires spurious
  interrupts when USBDDOS is loaded on a NEC µPD720101 or SiS 7001
  OHCI controller — Gap 5 (Legacy Support emulation not disabled
  after BIOS handoff).
* USBDDOS init silently fails to bring up the OHCI controller on
  OPTi 82C861 (FireLink) or older SiS OHCI silicon, even though the
  PCI enumeration succeeded — Gap 2 (`HcFmInterval` /
  `HcPeriodicStart` don't latch on first write).
* OHCI controller stops working after a `HcInterruptStatus.UE`
  (UnrecoverableError) interrupt is raised, with no recovery —
  Gap 1 (UE handler was empty upstream).
* Hot-plugged USB device on an OHCI root-hub port intermittently
  fails to enumerate on real hardware (works on second try) —
  Gap 8 (`POTPGT` wait after `SetPortPower` was missing).
* Plugging in any USB device whose class is unsupported by USBDDOS
  (audio class 1, printer class 7, vendor-specific 0xFF, etc.)
  triggers an assertion and hangs the system instead of skipping the
  device — fixed by the HCD-teardown commit (relevant to upstream
  [#7](https://github.com/crazii/USBDDOS/issues/7)).
* Building USBDDOS from source on Linux (DJGPP cross or Open Watcom
  v2) fails with `cannot find file` errors on `#include` directives —
  the Build commit (case-sensitivity fix, same shape as
  [stanwebber's #17](https://github.com/crazii/USBDDOS/issues/17)).

### Affected hardware (search aliases)

NEC µPD720101 · NEC µPD720100A · NEC µPD9210 · NEC USB 2.0 PCI card ·
ALi M5237 · ALi M1543C · ALi M1535+ · ULi M1573 ·
SiS 7001 · SiS 7002 · SiS 630 · SiS 5595 ·
OPTi 82C861 · OPTi FireLink ·
Intel ICH4 · ICH5 · ICH6 · ICH7 · ICH8 · ICH9 · ICH10 ·
Cougar Point · Panther Point · NM10 ·
VIA VT82C686A · VT82C686B · VT8231 · VT8233 · VT8235 · VT8237 ·
AMD-756 · AMD-768 · AMD SB700 · SB710 · SB800 · KT133A ·
Mac Mini 2011 · Sandy Bridge ThinkPad · Compaq Evo N600c ·
Toshiba Portege M200 · Toshiba Satellite 2410 · Lenovo ThinkPad T540p ·
Asus Eee PC · Intel Atom Cedarview · Gigabyte P67-DS3-B3 ·
ASRock ION 330.

### Related software / project terms

USBDDOS · USBDDOSP · USB driver DOS · DOS USB stack ·
FreeDOS · MS-DOS · DR-DOS · PC DOS ·
CWSDPMI · HDPMI32 · HDPMI32i · HX DPMI · JEMM386 · JEMMEX · QEMM386 ·
CTMOUSE · cutemouse · MS-DOS mouse ·
SBEMU · Sound Blaster emulator · RetroWav · RetroWave OPL3 · weeCee ·
USBDOS (Bret Johnson's separate driver — not the same project) ·
USBUHCIL · USBOHCIL · USBEHCIL · USBJSTIK · USBDRIVE ·
Rescue-D2 · UBCD · UnetbootIn · DESKTOP2 · FreeDOS rescue floppy ·
86Box · QEMU · Bochs · DOSBox · DOSBox-X · PCem · VirtualBox legacy ·
OHCI · UHCI · EHCI · xHCI · Open Host Controller Interface ·
Universal Host Controller Interface · Enhanced Host Controller Interface ·
USB 1.1 DOS driver · USB 2.0 DOS driver · USB DOS driver fix ·
DOS USB host controller · DOS USB hub support · DOS USB enumeration ·
DOS USB keyboard · DOS USB mouse · DOS USB mass storage ·
DOS USB rescue floppy · retro PC USB · vintage PC USB · retrocomputing ·
Pentium-era PC · 486-era PC · 386-era PC · Socket 7 USB · Slot 1 USB.

## License

GPL-2.0, inherited unchanged from upstream `crazii/USBDDOS`. See
[COPYING](./COPYING) for the full text. All credits in the upstream
README apply unchanged.

## Additional credits for fork work

* Linux USB host-controller code (`drivers/usb/host/ohci-hcd.c`,
  `pci-quirks.c`) — reference for Gaps 2, 3, and 6.
* Apple Darwin `IOUSBFamily` `AppleUSBOHCI` — reference for the
  NEC errata behind the (pending) Gap 7.
* Benjamin Lunt (fysnet), *FYSOS Book 8* — POTPGT semantics behind
  Gap 8.
* NEC µPD720101 User's Manual (S16336EJ4V0UM) and Datasheet (S16265E),
  ALi M1543C datasheet, SiS 630 datasheet — primary OHCI silicon
  references for Gaps 5, 6, and 8.

---

*The remainder of this README is the upstream content unchanged from
[crazii/USBDDOS@54345d0](https://github.com/crazii/USBDDOS/tree/54345d0).*

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

If you're using USB boot disk to boot the system, then ```/disk``` parameter is mandatory, because USBDDOS will take over from BIOS and make the BIOS's USB disk emulation unavailable, so the disk support need to be taken over too, otherwise the system drive will not function.  

Do not use ```LH/LOADHIGH``` for USBDDOS - A DPMI (or a DOS Extender) program is executed above 1M, so no need to do that. Actually if ```LH``` is used, then after TSR the PSP segment will resident in high memory with its' real mode data/code freed, which may generate fragmentations. A DPMI TSR also doesn't need to use assembly to strictly control/optimize the memory layout for TSR, as a common real mode program does.

# Credits
* RetroWaveLib from Sudomaker (folder renamed to RetroWav as 8.3 file name, and tiny code changes, for old compiler) https://github.com/SudoMaker/RetroWave
* This code is somehow based on an old code, usb-driver-under-dos: https://code.google.com/archive/p/usb-driver-under-dos/, with critital changes:
  * Multiple compiler support and code refactoring and major bug fixes.
  * DPMI support and wrapper added
  * IRQ handling added（polling changed to interrupt）
  
# How to Build
The build scripts support multiple hosts: Linux, Windows(WSL/MinGW), DOS/FreeDOS.  
Three set of toolchians are supported: DJGPP, Borland C++ 3.1, Open Watcom v2.  
Basically the Borland C++3.1 built executable has the same features as the Open Watcom build.  
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
|Instruction Set       |32-bit|16-bit          |16-bit       |
|Conventional Mem Usage|0K[\[1\]](README.md#note1)|12K[\[2\]](README.md#note2)|12K[\[2\]](README.md#note2) |

#### Note[1]:
USBDDOSP itself uses almost 0K (around 1K), but it depends on the DPMI host used, for HPDMI it is actually around 1K, and for CWSDPMI, the total usage is about 200K, most of them are used by CWSDPMI itself, after TSR.
#### Note[2]:
USBDDOS uses 4K memory if in real mode (without VCPI/EMM), 12K if in Virtual 8086 mode.

## DJGPP setup
* Download DGJPP from here: https://github.com/andrewwutw/build-djgpp  
* Setup environment, Run : ```. /djgpp/setenv``` or ```source /djgpp/setenv```  
* Make: ```make```  

For DOS host, download from here: https://www.delorie.com/djgpp/  and set the envs: ```set PATH=%PATH%;C:\DJGPP\BIN```, ```set DJGPP=C:\DJGPP\DJGPP.ENV```  
The Makefile will try to auto-detect DOS host, but may fail on FreeDOS, use ```make DOS=1``` to force DOS build.

## Borland C++ setup
The makefile uses default BC path: ```C:\BORLANDC``` for includes and libs.  
* Set exec path: ```set PATH=%PATH%;C:\BORLANDC\BIN```  
* Make: ```make -f Makefile.BC```
* If Borland C++ is installed on another location, i.e. C:\BC31,   
Then ```set PATH=%PATH%;C:\BC31\BIN```  and ```make -f Makefile.BC -DBCDIR=C:\BC31```

## Open Watcom setup
* Download Open Watcom from here: https://github.com/open-watcom/open-watcom-v2
* Set env: ```. /watcom/owsetenv.sh``` or ```source /watcom/owsetenv.sh```  
* Make: ```wmake -f Makefile.WC```

DOS setup: ```set PATH=%PATH%;C:\OW2\BINW```, ```set WATCOM=C:\OW2```, 
```set INCLUDE=C:\OW2\H```

# How to Debug
Add ```DEBUG=1``` on make commandline, i.e.  
 ```make DEBUG=1``` for DJGPP,  
  ```wmake -f Makefile.WC DEBUG=1``` for Open Watcom,  
  the built executable will have its logs/assertion enabled.  
  Use ```-DDEBUG``` for Borland C++'s make file.

# Requirements
* HIMEM.SYS or other XMS manager
* EMM386 4.46+/HDPMI32i(optional, for RetroWave driver)
* 80386 CPU or later
* MS-DOS (FreeDOS support is still in debugging)

# Features
The following drivers are added/planned:
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
