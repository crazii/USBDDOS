# Changelog

All notable changes to Netrunner01/USBDDOS are documented here. This
fork derives from [crazii/USBDDOS](https://github.com/crazii/USBDDOS)
commit `54345d0` (Feb 6, 2024). Upstream's own versioning is not
modified by this fork; the version strings below identify fork-side
iteration only.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/)
for the fork-side suffix.

## [1.0.0-alpha.1] — 2026-05-13

**Alpha designation rationale**: this release is `-alpha` because
real-hardware verification on the silicon each patch targets is
pending — NOT because the code is unstable or broken. All patches
build clean with `-Werror` on both Open Watcom v2 and DJGPP gcc
12.2.0, and pass a 7-test QEMU regression suite. Patch designs are
derived from authoritative sources (USB-IF specs, Linux
`pci-quirks.c` and `ohci-hcd.c`, Apple Darwin `IOUSBFamily`, FYSOS
Book 8, and chip vendor datasheets). The verification gap is
specifically the proof that each chip-quirk fix behaves correctly
on the real silicon it targets — QEMU emulates spec-compliant USB
devices, and the patched paths exist precisely because real silicon
deviates from spec.

Promotion criteria: this release moves to `-beta` when at least one
independent real-hardware test report exists per patched gap.
General-availability `v1.0.0` (drop the `-alpha`/`-beta` suffix)
when each gap has at least one PASS report on its target silicon.

### Added (chip-quirk fixes)
* **Gap 1** — OHCI `UnrecoverableError` recovery per OHCI 1.0a
  §5.3.1.1. The UE handler in `OHCI_ISR` was empty upstream
  (`//return TRUE;`); now performs full HC reset and operational-
  register restore. Files: `USBDDOS/HCD/ohci.c`.
* **Gap 2** — INITRESET-equivalent verify-and-retry for
  `HcFmInterval` / `HcPeriodicStart` latching on OPTi 82C861
  (FireLink) and certain SiS OHCI silicon. Mirrors Linux's
  `OHCI_QUIRK_INITRESET`. Files: `USBDDOS/HCD/ohci.c`.
* **Gap 3** — Bounded SMM-handoff wait loop (~1 s cap, 20 × 50 ms
  poll). Replaces the upstream unbounded `while` that hangs on
  BIOSes which never release SMM ownership. Mirrors Linux's
  `quirk_usb_handoff_ohci` pattern. Files: `USBDDOS/HCD/ohci.c`.
* **Gap 4** — Fixed invalid `ClearPortFeature(PORT_RESET)` hub-port-
  reset spec violation in `USBDDOS/CLASS/hub.c`. Per USB 2.0
  Table 11-17, `PORT_RESET` (feature 4) is a `SetPortFeature` selector
  only; the post-reset change indicator is cleared via
  `ClearPortFeature(C_PORT_RESET)` (feature 20) instead. Fix applied
  at both call sites (lines 77 and 189). Wait loop also bounded with
  a 100-iteration timeout (~500 ms cap). Files: `USBDDOS/CLASS/hub.c`.
* **Gap 5** — Disable OHCI Legacy Support emulation after BIOS
  handoff (one-shot `HceControl = 0` write at offset 0x100). Prevents
  PS/2-style emulation from interfering with HCD operation on NEC
  µPD720101, SiS 7001, and similar Legacy-Support-capable silicon.
  Universally safe per spec: chips without Legacy Support treat the
  offset as reserved/RAZ. Files: `USBDDOS/HCD/ohci.c`,
  `USBDDOS/HCD/ohci.h`.
* **Gap 6** — Skip `HcFmInterval` access on ALi/ULi M5237 OHCI
  silicon (PCI ID `10B9:5237`) — read/restore causes hard system
  lockup. Mirrors Linux's `no_fminterval` flag in
  `quirk_usb_handoff_ohci`. Default `OHCI_FI_DEFAULT = 0x2EDF`
  substituted. Files: `USBDDOS/HCD/ohci.c`.
* **Gap 8** — Wait `POTPGT × 2 ms` after `SetPortPower` per OHCI
  1.0a §7.4.1. POTPGT is read from `HcRhDescriptorA` bits 24-31;
  the previously missing wait caused intermittent hot-plug
  enumeration failures on NEC µPD720101 (datasheet POTPGT default
  0x0F = 30 ms). Files: `USBDDOS/HCD/ohci.c`.

### Added (defensive fixes)
* `HCD_RemoveDevice` teardown made defensive. Strict `assert(result)`
  calls in the three-step cleanup path replaced with logged best-
  effort cleanup. Each step now runs unconditionally; device-list
  removal happens regardless of individual step success. Fixes a
  latent device-list-leak in release builds (NDEBUG strips the
  asserts but the short-circuit `result = result && ...` chain
  still skipped subsequent cleanup steps). Resolves the symptoms
  reported in crazii/USBDDOS#7. Files: `USBDDOS/HCD/hcd.c`.

### Added (developer-facing)
* COM1 logging path in `_LOG` enabled (was gated off with `#if 0`
  upstream). Required for headless emulator regression and
  serial-console real-hardware debugging. Behavior unchanged on
  release builds (NDEBUG strips `_LOG` entirely). Resolves
  crazii/USBDDOS#19. Files: `USBDDOS/dbgutil.c`.
* Case-correct `#include` paths for cross-build on Linux. Same fix
  shape as `stanwebber/USBDDOS` (8 commits dated April 25, 2025);
  independently derived, credit to stanwebber for the prior
  independent work and for filing crazii/USBDDOS#17. Files:
  `USBDDOS/HCD/hcd.h`, `RetroWav/Board/opl3.c`, `RetroWav/Board/opl3.h`,
  `RetroWav/Platform/dos_cdc.h`, `RetroWav/Protocol/serial.c`,
  `RetroWav/retrowav.c`, `USBDDOS/dbgutil.c`, `main.c`.
* `README.md` fork-front-page section: purpose, scope, "who this
  fork is for" symptom keys, common-symptom verbatim error strings,
  affected-hardware aliases, related-software terms. Designed for
  search-engine discoverability by users hunting their own log
  messages.

### Tooling
* 7-test QEMU regression harness (`run-test.sh`, distributed
  separately): `ohci-init`, `ohci-kbd`, `ohci-msc`, `uhci-init`,
  `ehci-init`, `ohci-hub-hid` (Gap 4 reproducer), `ohci-audio`
  (unsupported-class HCD-teardown exerciser).

### Known limitations (carried into next iteration)
* No real-hardware test runs yet. See Status section in `README.md`.
* **Gap 7** — NEC `kErrataNECIncompleteWrite` write-1-to-clear retries
  on `HcRhPortStatus` is sketched (from Apple Darwin
  `AppleUSBOHCI_RootHub.cpp`) but not yet patched. Affects µPD720101
  under specific load patterns.
* **EHCI USBLEGSUP/USBLEGCTLSTS BIOS handoff** is not implemented.
  Gap 3 covers the OHCI side; the EHCI-side analogue (which Linux's
  `quirk_usb_disable_ehci` covers, including the
  `EHCI_USBLEGCTLSTS_SOOE` SMI-on-Ownership-Change-Enable bit) is
  the most likely root cause of unresolved upstream hangs on Intel
  Cougar Point / Panther Point platforms (crazii/USBDDOS#10, #20,
  and the Gigabyte P67-DS3-B3 thread on VOGONS).
* **xHCI** support is upstream-planned and out of scope for this
  fork.

### Build details
* **Open Watcom v2 debug build**: `usbddos.exe`, 94,380 bytes,
  bit-stable across rebuilds at this commit.
* **DJGPP gcc 12.2.0 release build**: `usbddosp.exe`, 207,872 bytes.
  Byte-content varies across rebuilds — the build embeds
  `git rev-parse --short HEAD` into the binary via `-DUSBDDOS_BUILD`,
  so the 7-character commit-hash string differs between source-
  identical rebuilds at different commits. The non-hash code bytes
  are identical.

### Reference upstream commit
* `54345d0` — `BC/WC: bugfix on loading ivt` (Feb 6, 2024). This is
  the commit on `master` that this fork's series rebases onto.
  Verified accessible via `git ls-remote https://github.com/crazii/USBDDOS.git`.

[1.0.0-alpha.1]: https://github.com/Netrunner01/USBDDOS/releases/tag/v1.0.0-alpha.1
