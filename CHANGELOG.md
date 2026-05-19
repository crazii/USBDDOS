# Changelog

All notable changes to Netrunner01/USBDDOS are documented here. This
fork derives from [crazii/USBDDOS](https://github.com/crazii/USBDDOS);
the initial alpha was based on upstream `54345d0` (Feb 6, 2024), and
subsequent releases track upstream as the fork's patches are merged
in. Upstream's own versioning is not modified by this fork; the
version strings below identify fork-side iteration only.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/)
for the fork-side suffix.

## [1.0.0-alpha.2] — 2026-05-15

**This is a housekeeping release.** No new code, no functional
changes, no bug fixes. The alpha.2 binaries are byte-identical (Watcom)
or near-identical (DJGPP differs by 7 bytes of embedded build-hash) to
the alpha.1 binaries. If alpha.1 is already running on your system, do
not update — there is no reason to.

**Why cut a release at all, then?** Between May 13 and May 15, 2026,
upstream `crazii/USBDDOS` merged every patch from this fork's
`v1.0.0-alpha.1` patch series (PRs #22–#30). The patches now live at
upstream `master` (`b1308fb`) and are no longer carried as fork-side
cherry-picks. This release exists solely to refresh the binaries' build
provenance so they map to a public upstream commit rather than to a
fork-internal tag — useful for anyone wanting to verify "these binaries
correspond to crazii/USBDDOS at commit X" without trusting fork-side
state.

**Alpha designation (carried forward unchanged from alpha.1)**:
real-hardware verification on the silicon the patches target is still
pending. The alpha label remains accurate because the field-validation
criterion has not changed and is not affected by where the source
lives. Promotion to beta or v1.0.0 still requires at least one
independent real-hardware test report per patched gap.

### Changed
* **Source provenance**: binaries are now built from upstream master
  at commit `b1308fb`, not from fork-side cherry-picks. Anyone can
  `git clone https://github.com/crazii/USBDDOS.git && git checkout b1308fb`
  and build a byte-identical Watcom binary.
* **Build hash embedded in DJGPP binary**: `b1308fb, DJGPP` (was
  `afb2b84, DJGPP` in alpha.1). This is the only differing region
  between alpha.1 and alpha.2 DJGPP binaries.

### Upstream merge order (for the record)
1. PR #17 (stanwebber: case-correct `#include` paths) — merged
   earlier; fork's own case-fix patch deferred in favor of this one.
2. PR #22 (debug: COM1 logging)
3. PR #23 (HCD defensive teardown)
4. PR #24 (Gap 4: hub `ClearPortFeature(PORT_RESET)` spec violation)
5. PR #25 (Gap 8: POTPGT wait)
6. PR #26 (Gap 2: INITRESET-equivalent retry)
7. PR #27 (Gap 3: SMM-handoff bound)
8. PR #28 (Gap 1: OHCI UnrecoverableError recovery)
9. PR #29 (Gap 6: ALi M5237 HcFmInterval lockup avoidance)
10. PR #30 (Gap 5: OHCI Legacy Support emulation disable)

### Unchanged
* Functional code: all patches landed unmodified. The Open Watcom v2
  debug build is byte-identical to alpha.1's (MD5
  `39962c2092961ba679467ba46ab24c43`, 94,380 bytes), confirming that
  Crazii merged each patch as-submitted and that stanwebber's
  case-fix has no effect on Windows-built (case-insensitive)
  binaries.

### Build details
* **Open Watcom v2 debug build**: `usbddos.exe`, 94,380 bytes,
  MD5 `39962c2092961ba679467ba46ab24c43`, bit-stable across rebuilds.
* **DJGPP gcc 12.2.0 release build**: `usbddosp.exe`, 207,872 bytes,
  MD5 `2b1b0221f338c59041e7e6ec18827ef9`. Byte-content of all non-hash
  regions identical to alpha.1.

### Reference upstream commit
* `b1308fb` — `Merge pull request #30 from netrunner01/fix/ohci-legacy-support-disable`
  (May 15, 2026). All of the fork's alpha.1 patch series is reachable
  from this commit.

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
