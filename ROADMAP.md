# T2-Application Firmware Improvement and Advancement Roadmap

**Project:** STM32F207 Dental X-ray Imaging Device (Pano/CBCT/Ceph)
**Date:** 2026-03-24
**Baseline:** master branch, post-architecture redesign (new modules created, not yet integrated)
**Hardware:** STM32F207VGTx (120MHz Cortex-M3, 1MB Flash, 128KB SRAM -- 112KB usable per linker)

---

## Current State Summary

| Category | Old (Active in Build) | New (Created, Not Integrated) |
|----------|----------------------|-------------------------------|
| Motor control | `motor.c` (4,912 lines, float in ISR) | `motor_ctrl.c` + `motor_ctrl.h` (LUT ISR, integer-only) |
| TMC2660 SPI | Embedded in `motor.c` (3 near-identical functions) | `tmc2660.c` + `tmc2660.h` (table-driven) |
| ISR handlers | `isr.c` (778 lines, printUart in ISR, float) | `isr_new.c` + `isr_new.h` (lightweight, flag-based) |
| Safety | None (no IWDG, printUart in emergency ISR) | `safety.c` + `safety.h` (IWDG, ISR-safe emergency) |
| Serial comms | `serial.c` (9,059 lines, monolithic) | Split: `uart_hw.c`, `msg_protocol.c`, `cmd_system.c`, `cmd_pano.c`, `cmd_ct.c`, `cmd_scan.c`, `cmd_geoalign.c`, `cmd_geoalign_axes.c`, `cmd_geoalign_cal.c`, `cmd_eeprom_diag.c`, `cmd_calibration.c` |
| Shared vars | No `volatile`, `g_bTiltStatus` multiply defined | `shared_vars.h` (all volatile, single definition point) |
| Debug logging | `printUart()` from ISR context | `debug_log.c` + `debug_log.h` (ring buffer, ISR-safe) |
| Compat layer | N/A | `motor_compat.c` + `motor_compat.h` (old API -> new API bridge) |

**Build system:** STM32CubeIDE `.cproject` (Eclipse CDT managed build, arm-none-eabi-gcc 7-2018-q2)
**Linker script:** `/home/issacs/work/projects/T2-Application/stm32_flash.ld` -- Flash origin 0x08020000 (128KB bootloader offset), 112KB RAM
**Backup files:** 16 `.bak` files present in `src/` and `inc/`

---

## Phase 1: Stabilization (Week 1-2)

**Goal:** Get new modules compiling alongside old ones without changing runtime behavior.

### Step 1.1: Create a standalone CMake/Makefile for host-side compilation check
- **What:** Create a minimal `Makefile.host` that compiles all `.c` files (old + new) with `arm-none-eabi-gcc` on a CI/development machine, using the same defines from `.cproject` line 54-63 (`STM32F2XX`, `USE_TABLET_PC`, `USE_MOTOR_GANTRY_MS`, `USE_I2C_EEPROM`, `USE_MOTOR_CHINREST_TS`, `USE_MOTOR_CHINREST_HOR`, `USE_MOTOR_CHINREST_VER`, `USE_CT_STITCH_MODE`, `USE_STDPERIPH_DRIVER`).
- **Why:** The `.cproject` XML build system is opaque and not scriptable. A parallel Makefile enables automated verification outside the IDE.
- **Files:** Create `/home/issacs/work/projects/T2-Application/Makefile` (new)
- **Risk:** LOW
- **Dependencies:** None
- **Effort:** 4 hours
- **Verification:** `make -f Makefile all` produces `.o` files for every source without errors

### Step 1.2: Add new source files to the STM32CubeIDE build
- **What:** Add the following files to the `.cproject` managed build:
  - `src/motor_ctrl.c`
  - `src/tmc2660.c`
  - `src/safety.c`
  - `src/debug_log.c`
  - `src/uart_hw.c`
  - `src/motor_compat.c`
  - `src/msg_protocol.c`
  - `src/cmd_system.c`, `src/cmd_pano.c`, `src/cmd_ct.c`, `src/cmd_scan.c`
  - `src/cmd_geoalign.c`, `src/cmd_geoalign_axes.c`, `src/cmd_geoalign_cal.c`
  - `src/cmd_eeprom_diag.c`, `src/cmd_calibration.c`
  - Do NOT yet add `src/isr_new.c` (duplicate symbol conflict with `src/isr.c`)
- **Why:** Source files exist on disk but are not in the Eclipse managed build. The `.cproject` XML must include them, or they must be placed in the source folder discovery path.
- **Files:** Modify `/home/issacs/work/projects/T2-Application/.cproject`
- **Risk:** MEDIUM -- symbol conflicts between old serial.c and new cmd_*.c files if both define same functions
- **Dependencies:** Step 1.1 (parallel Makefile for validation)
- **Effort:** 6 hours
- **Verification:** Build in STM32CubeIDE or `make` -- zero compilation errors, zero linker errors

### Step 1.3: Resolve duplicate symbol conflicts
- **What:** Identify and fix any linker-level duplicate symbols between:
  - Old `serial.c` vs new `msg_protocol.c` + `cmd_*.c` + `uart_hw.c` -- likely the serial split functions exist in both places simultaneously. Either guard old serial.c functions with `#ifndef USE_NEW_SERIAL` or exclude old `serial.c` from build and verify new modules cover all functionality.
  - Old `isr.c` ISR handler names vs `isr_new.c` -- both define `TIM8_CC_IRQHandler`, `EXTI9_5_IRQHandler`, etc. Cannot link both. Keep `isr.c` in build for now.
  - `g_bTiltStatus` in `extern.h` (non-extern, CODE_REVIEW critical #1) vs `shared_vars.h` (extern volatile)
- **Files:**
  - `/home/issacs/work/projects/T2-Application/inc/extern.h` (fix `g_bTiltStatus` to `extern`)
  - `/home/issacs/work/projects/T2-Application/src/serial.c` (conditionally exclude duplicate functions)
  - `/home/issacs/work/projects/T2-Application/src/system.c` (define `g_bTiltStatus` storage)
- **Risk:** HIGH -- touching `extern.h` affects every translation unit. `g_bTiltStatus` fix is safety-critical (tilt interlock was broken).
- **Dependencies:** Step 1.2
- **Effort:** 8 hours
- **Verification:**
  - Build succeeds with zero warnings about multiply-defined symbols
  - `arm-none-eabi-nm` output shows single definition for each global
  - On hardware: tilt interlock test -- command TILT_ON, verify all .c files see the same value

### Step 1.4: Fix critical bugs from CODE_REVIEW (non-architecture changes)
- **What:** Fix the 6 CRITICAL issues identified in `CODE_REVIEW.md` that do not require architecture migration:
  1. `extern.h:56` -- `g_bTiltStatus` non-extern (fix in Step 1.3)
  2. `system.h:116, main.c:68` -- `CurCaptureMode` missing `volatile` (already fixed via `shared_vars.h` if included, but old code paths via `extern.h` may bypass it)
  3. `isr.c:782` -- `Sensor_P.CaptureCcr[ArrayIndex]` unbounded array access -- add bounds check
  4. `serial.c:1088-1141` -- UART buffer overflow -- add count guard
  5. `emergency.c:167` -- `printUart()` in emergency ISR -- set flag only (partially addressed by `isr_new.c` but `isr.c` still active)
  6. `boot.c:100` -- Flash write sizeof/word-count mismatch -- fix loop
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/isr.c` lines ~782 (bounds check)
  - `/home/issacs/work/projects/T2-Application/src/serial.c` lines ~1088-1141 (buffer guard)
  - `/home/issacs/work/projects/T2-Application/src/emergency.c` line ~167 (remove printUart)
  - `/home/issacs/work/projects/T2-Application/src/boot.c` line ~100 (sizeof fix)
  - `/home/issacs/work/projects/T2-Application/inc/extern.h` (volatile on shared vars)
- **Risk:** HIGH -- these are safety-critical bugs in a medical device. The boot.c flash corruption bug could brick devices.
- **Dependencies:** Step 1.3
- **Effort:** 12 hours (including testing each fix individually)
- **Verification:**
  - Pano capture with ArrayIndex at boundary values -- no Hard Fault
  - UART stress test with oversized messages -- no buffer overflow
  - Emergency switch test -- no UART deadlock, flag-based only
  - Bootloader flash write/read back verification

### Step 1.5: Fix HIGH priority issues from CODE_REVIEW
- **What:** Address the 6 HIGH issues:
  7. `isr.c` motor ISRs too long -- defer to Phase 2/3 (architecture migration)
  8. `isr.c:351` printUart in CAN ISR -- add flag-based CAN processing (already in main.c:318-321, just need to verify `isr.c` CAN handler matches)
  9. `timer.h:26-29` macro precedence bug and integer division -- fix macros
  10. `can.c:253` CAN filter mask 0x0000 -- set proper filter
  11. `motor.c:1491,1644` TMC2660 bounds check -- add guards
  12. `tube.c:168` `Tube.bXrayOnOff` init to TRUE -- change to FALSE
- **Files:**
  - `/home/issacs/work/projects/T2-Application/inc/timer.h` lines ~26-29
  - `/home/issacs/work/projects/T2-Application/src/can.c` line ~253
  - `/home/issacs/work/projects/T2-Application/src/motor.c` lines ~1491, ~1644
  - `/home/issacs/work/projects/T2-Application/src/tube.c` line ~168
- **Risk:** HIGH -- CAN filter fix prevents message injection. X-ray init fix prevents unintended exposure.
- **Dependencies:** Step 1.4
- **Effort:** 8 hours
- **Verification:**
  - CAN bus analyzer: send invalid CAN ID, verify it is rejected
  - Power-on: verify Tube.bXrayOnOff == FALSE before explicit enable
  - Timer macro: unit test on host with known values
  - TMC2660: fuzz test with out-of-range motor indices

### Step 1.6: Set up Unity test framework for host-side unit tests
- **What:** Create a `test/` directory with Unity framework, a test runner, and initial test files for:
  - `test_motor_ctrl.c` -- profile builder (trapezoidal, arch), Q16.16 math
  - `test_safety.c` -- flag management, error bitmask operations
  - `test_debug_log.c` -- ring buffer wrap-around, overflow
  - `test_timer_macros.c` -- macro precedence, edge cases
  - HAL mock layer: `test/mocks/stm32f2xx_mock.h` with stubbed TIM, GPIO, SPI, IWDG, RCC, CAN functions
- **Why:** Zero unit tests currently. Medical device firmware requires documented test evidence.
- **Files:**
  - Create `/home/issacs/work/projects/T2-Application/test/` directory
  - Create `/home/issacs/work/projects/T2-Application/test/unity/` (Unity source)
  - Create `/home/issacs/work/projects/T2-Application/test/mocks/stm32f2xx_mock.h`
  - Create test files per module
- **Risk:** LOW
- **Dependencies:** Step 1.1 (host Makefile)
- **Effort:** 16 hours
- **Verification:** `make test` runs all tests, green output, coverage report generated

### Phase 1 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 1.1 Host Makefile | 4 | LOW |
| 1.2 Add sources to build | 6 | MEDIUM |
| 1.3 Duplicate symbol resolution | 8 | HIGH |
| 1.4 Critical bug fixes (6) | 12 | HIGH |
| 1.5 High priority fixes (6) | 8 | HIGH |
| 1.6 Unity test setup | 16 | LOW |
| **Total** | **54** | |

**Phase 1 Exit Criteria:**
- [ ] All new and old sources compile together (zero errors, zero linker warnings)
- [ ] All 6 CRITICAL bugs fixed and individually verified on hardware
- [ ] All 6 HIGH bugs fixed
- [ ] Unity test framework operational with initial test coverage for motor_ctrl profile builder
- [ ] Flash/RAM size report: `.text` and `.bss` within budget (1MB Flash, 112KB SRAM)

---

## Phase 2: Motor Migration (Week 3-5)

**Goal:** Replace old motor.c ISR-computed CCR with new LUT-based motor_ctrl.c, one motor at a time.

**SRAM Budget Alert:** `motor_ctrl.h` states 8 motors x 4096 x 2 bytes = 64KB for profile buffers alone. Total SRAM is 112KB. The old motor.c Motor_Typedef structs also consume significant SRAM. During migration both coexist.
- **Mitigation:** Reduce `PROFILE_BUF_SIZE` to 2048 (32KB) during migration period, increase later per-motor as needed. Or migrate fully before enabling all 8 buffers.

### Step 2.1: Verify motor_ctrl.c profile math against old motor.c

- **What:** For each motor, extract the old CCR computation from `isr.c` (Motor_GetNextRunCCR, Motor_GetNextOrgCCR using float `+ 0.5`) and the new `MotorCtrl_BuildTrapProfile()` Q16.16 path. Compare outputs for known input parameters (from existing motor config constants in `motor.c`). Run this as a host-side test.
- **Why:** The integer-only profile builder must produce identical or within-1-LSB CCR values compared to the float path.
- **Files:**
  - `/home/issacs/work/projects/T2-Application/test/test_profile_equivalence.c` (new)
  - `/home/issacs/work/projects/T2-Application/src/motor_ctrl.c` (read for profile builder)
  - `/home/issacs/work/projects/T2-Application/src/motor.c` (read for old Motor_GetNextRunCCR)
- **Risk:** MEDIUM -- if Q16.16 math diverges by >1 CCR count at high frequencies, motor dynamics change
- **Dependencies:** Phase 1 complete
- **Effort:** 12 hours
- **Verification:** Host test: for each motor configuration, max CCR difference <= 1 count for all step indices

### Step 2.2: Migrate Motor_R (Rotation) first
- **What:** Motor_R is the highest-priority motor (TIM8, NVIC priority 0/0) used in ALL capture modes. Migrate by:
  1. In `pano_capture.c`, `ct_capture.c`, `scan_capture.c`: replace `Motor_MoveAbsolutePosition(&Motor_R, ...)` calls with `Motor_MoveAbsolutePosition_New(&Motor_R, ...)` via the compat layer.
  2. Ensure arch trajectory LUT (from `arch.c`) is loaded via `MotorCtrl_BuildArchProfile()` for panoramic capture.
  3. The ISR routing stays in old `isr.c` for now -- only the profile computation changes.
- **Why:** Motor_R exercises both trapezoidal (CT 365 degree, Ceph 90 degree) and arch (Pano) profiles. Testing here validates both code paths.
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/pano_capture.c`
  - `/home/issacs/work/projects/T2-Application/src/ct_capture.c`
  - `/home/issacs/work/projects/T2-Application/src/scan_capture.c`
  - `/home/issacs/work/projects/T2-Application/src/motor_compat.c`
- **Risk:** HIGH -- Motor_R controls rotation arm holding X-ray tube. Wrong speed profile = patient collision.
- **Dependencies:** Step 2.1 (profile equivalence verified)
- **Effort:** 16 hours
- **Verification:**
  - **Oscilloscope checkpoint 1:** Compare TIM8 CC3 step pulse frequency ramp (old vs new) during a Pano standard arch trajectory. Capture must show identical frequency envelope (+/- 1%).
  - **Oscilloscope checkpoint 2:** CT 365 degree rotation -- verify trapezoidal profile: accel ramp time, cruise frequency, decel ramp time match old behavior.
  - **Functional test:** Complete a full Pano capture with X-ray (phantom). Image quality unchanged.
  - **Functional test:** Complete a full CBCT capture (5x5 FOV). Frame count = 400.
  - **Rollback:** Revert `_New()` calls back to old `Motor_MoveAbsolutePosition()`. Compat layer makes this a single-line change per call site.

### Step 2.3: Migrate Motor_V (Vertical)
- **What:** Replace Motor_V calls in capture modules. Motor_V is trapezoidal only (no arch).
- **Test with:** Pano capture (V moves vertically during arch). Verify vertical position at capture start/end matches expected.
- **Files:** `pano_capture.c`, `ct_capture.c`
- **Risk:** MEDIUM -- vertical position error affects image alignment but not safety
- **Dependencies:** Step 2.2 verified
- **Effort:** 8 hours
- **Verification:**
  - Oscilloscope: TIM1 CC1 step count during Pano capture equals expected step count from motor config
  - Functional: vertical position matches with dial indicator

### Step 2.4: Migrate Motor_C (Collimator) and Motor_S (Slit)
- **What:** These two motors are Ceph-specific (C = detector scan, S = slit scan). They run synchronized during scan capture.
- **Test with:** Ceph Full_Lateral scan mode.
- **Files:** `scan_capture.c`
- **Risk:** HIGH -- synchronized C+S motors. Use `MotorCtrl_StartSync()` for minimal skew.
- **Dependencies:** Step 2.3 verified
- **Effort:** 12 hours
- **Verification:**
  - Oscilloscope: TIM2 (Motor_C) and TIM10 (Motor_S) start within 1us of each other (sync start)
  - Functional: Ceph scan image -- no banding or distortion from sync error
  - Test all 8 Ceph sub-modes: Full_Lateral, Lateral, Carpus, PA, AP, Waters, SVM, EarLod

### Step 2.5: Migrate Motor_T (Temple/Chinrest TS)
- **What:** Temple support motor. Used for patient positioning.
- **Test with:** Temple push/release command from PC application.
- **Files:** `serial.c` or `cmd_system.c` (temple command handler)
- **Risk:** LOW -- non-capture motor, positioning only
- **Dependencies:** Step 2.3
- **Effort:** 4 hours
- **Verification:** Temple extends/retracts to correct position. Limit switch stops correctly.

### Step 2.6: Migrate Motor_A (Gantry)
- **What:** Gantry motor moves the entire arm assembly. Ceph-specific.
- **Test with:** Ceph capture -- gantry moves to scan position.
- **Files:** `scan_capture.c`
- **Risk:** MEDIUM -- heavy mechanical load. Profile mismatch could stall motor.
- **Dependencies:** Step 2.4 (tested with Ceph)
- **Effort:** 6 hours
- **Verification:**
  - Oscilloscope: TIM9 step frequency profile matches old behavior
  - Functional: gantry reaches scan position without stall

### Step 2.7: Migrate Motor_CNS (Chinrest vertical) and Motor_CWE (Chinrest horizontal)
- **What:** Patient positioning motors. Used in all capture modes.
- **Test with:** Patient positioning sequence before any capture.
- **Files:** `pano_capture.c`, `ct_capture.c`, `scan_capture.c`
- **Risk:** LOW -- slow-speed positioning motors
- **Dependencies:** Step 2.5
- **Effort:** 6 hours
- **Verification:** Chinrest moves to correct position. Hall sensor feedback matches expected.

### Step 2.8: Remove old Motor_Typedef ISR computation code paths
- **What:** After all 8 motors are migrated, remove `Motor_GetNextRunCCR()`, `Motor_GetNextOrgCCR()`, and float-based CCR computation from `motor.c`. Keep motor configuration (pin defines, init sequences) until ISR migration.
- **Files:** `/home/issacs/work/projects/T2-Application/src/motor.c`
- **Risk:** MEDIUM -- must not remove anything still referenced
- **Dependencies:** Steps 2.2-2.7 all verified
- **Effort:** 6 hours
- **Verification:** Build succeeds. `arm-none-eabi-size` shows reduced `.text`. All 3 capture modes pass.

### Phase 2 Summary
| Item | Effort (hrs) | Risk | Capture Mode Test |
|------|-------------|------|-------------------|
| 2.1 Profile equivalence test | 12 | MEDIUM | Host-only |
| 2.2 Motor_R migration | 16 | HIGH | Pano + CBCT + Ceph |
| 2.3 Motor_V migration | 8 | MEDIUM | Pano + CBCT |
| 2.4 Motor_C + Motor_S migration | 12 | HIGH | Ceph (all 8 sub-modes) |
| 2.5 Motor_T migration | 4 | LOW | Temple push/release |
| 2.6 Motor_A migration | 6 | MEDIUM | Ceph |
| 2.7 Motor_CNS + Motor_CWE migration | 6 | LOW | Patient positioning |
| 2.8 Remove old CCR computation | 6 | MEDIUM | All modes regression |
| **Total** | **70** | | |

**Phase 2 Exit Criteria:**
- [ ] All 8 motors use `MotorCtrl_BuildTrapProfile()` or `MotorCtrl_BuildArchProfile()`
- [ ] Zero float operations remain in any motor ISR path
- [ ] Oscilloscope verification for Motor_R (Pano arch + CT trap) and Motor_C+S (Ceph sync)
- [ ] Full regression: Pano standard/child/sinus/TMJ, CT 5x5/8x8/15x15, Ceph all sub-modes
- [ ] SRAM usage within budget (check with `arm-none-eabi-size`)

---

## Phase 3: ISR Migration (Week 5-6)

**Goal:** Switch from `isr.c` to `isr_new.c` -- all ISR handlers use lightweight, flag-based approach.

### Step 3.1: Prepare ISR switchover
- **What:**
  1. Verify `isr_new.c` covers every IRQ vector that `isr.c` defines (13 handlers total)
  2. Check for any ISR handlers defined in OTHER files (e.g., `emergency.c` has `EXTI9_5_IRQHandler`, `stm32f2xx_it.c` may have default handlers)
  3. Ensure `stm32f2xx_it.c` does not conflict (it typically has weak/empty default handlers)
  4. Create `#define USE_ISR_NEW` build flag for switching
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/isr_new.c`
  - `/home/issacs/work/projects/T2-Application/src/isr.c`
  - `/home/issacs/work/projects/T2-Application/src/stm32f2xx_it.c`
  - `/home/issacs/work/projects/T2-Application/src/emergency.c` (has its own EXTI handler)
- **Risk:** HIGH -- duplicate weak symbol override. If `emergency.c` defines `EXTI9_5_IRQHandler` and `isr_new.c` also defines it, linker must pick one. Need to guard `emergency.c` handler.
- **Dependencies:** Phase 2 complete (motor ISR handlers in isr_new.c use MotorCtrl_ISR_Handler)
- **Effort:** 8 hours
- **Verification:** `arm-none-eabi-nm -C *.o | grep IRQHandler` shows exactly one definition per vector

### Step 3.2: Switch build from isr.c to isr_new.c
- **What:**
  1. Exclude `isr.c` from build
  2. Include `isr_new.c` in build
  3. Guard `emergency.c` EXTI handler with `#ifndef USE_ISR_NEW`
  4. Build and fix any missing symbol references
- **Files:**
  - `/home/issacs/work/projects/T2-Application/.cproject` (exclude isr.c, include isr_new.c)
  - `/home/issacs/work/projects/T2-Application/src/emergency.c` (guard EXTI handler)
- **Risk:** HIGH -- single switch point, all ISRs change simultaneously
- **Dependencies:** Step 3.1
- **Effort:** 4 hours
- **Verification:** Build succeeds. Binary size comparison (should be smaller).

### Step 3.3: Verify ISR timing with logic analyzer
- **What:** For each ISR, measure entry-to-exit time:
  - Motor ISRs (8): target < 500ns (30 cycles at 120MHz). Measure by toggling a spare GPIO at ISR entry/exit (temporary instrumentation).
  - CAN ISR: target < 2us. Should only read message + set flag.
  - UART ISRs: target < 1us. Single byte read + parser call.
  - Timer ISRs (TIM6, TIM7): target < 200ns. Counter increment only.
  - Emergency ISR: target < 2us. Motor stop + GPIO + flag.
- **Files:** Temporary instrumentation in `/home/issacs/work/projects/T2-Application/src/isr_new.c` (GPIO toggle at entry/exit, remove after measurement)
- **Risk:** LOW (measurement only)
- **Dependencies:** Step 3.2
- **Effort:** 8 hours
- **Verification:** All ISRs within target timing. Document measurements.

### Step 3.4: Verify NVIC priorities match original configuration
- **What:** Dump all NVIC priority register values and compare against ARCHITECTURE.md timer allocation table:
  - Motor timers: preemption priority 0 (highest) for Motor_R, 0 or 1 for others
  - TIM4 (Hall sensor): priority 3/1
  - TIM6/TIM7: priority 3/0
  - UART1/UART2: check existing system.c init
  - CAN2: check existing can.c init
  - EXTI5 (emergency): must be highest possible
- **Files:** `/home/issacs/work/projects/T2-Application/src/system.c` (NVIC init code)
- **Risk:** MEDIUM -- wrong priority causes missed interrupts or priority inversion during X-ray exposure
- **Dependencies:** Step 3.2
- **Effort:** 4 hours
- **Verification:** NVIC priority dump matches specification. No priority inversion possible between motor ISRs and sensor ISRs during capture.

### Step 3.5: Verify CAN main-loop processing
- **What:** With `isr_new.c` active, the CAN ISR sets `g_can_rx_flag` and main loop calls `CAN_ParseMessage()` (main.c:318-321). Verify:
  1. Generator status messages are received and processed
  2. Collimator responses are received
  3. Tube temperature readback works
  4. No message loss under burst conditions
- **Files:** `/home/issacs/work/projects/T2-Application/src/main.c` (CAN polling), `/home/issacs/work/projects/T2-Application/src/can.c`
- **Risk:** MEDIUM -- main loop iteration time determines CAN response latency
- **Dependencies:** Step 3.2
- **Effort:** 6 hours
- **Verification:**
  - CAN bus analyzer: send 100 messages in 1 second, verify all processed
  - Generator init sequence completes without timeout
  - Collimator auto-align responds correctly

### Phase 3 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 3.1 ISR switchover preparation | 8 | HIGH |
| 3.2 Build switch isr.c -> isr_new.c | 4 | HIGH |
| 3.3 ISR timing measurement | 8 | LOW |
| 3.4 NVIC priority verification | 4 | MEDIUM |
| 3.5 CAN main-loop processing | 6 | MEDIUM |
| **Total** | **30** | |

**Phase 3 Exit Criteria:**
- [ ] `isr.c` excluded from build, `isr_new.c` active
- [ ] All motor ISRs < 500ns measured
- [ ] CAN message processing verified (no message loss under burst)
- [ ] NVIC priorities documented and verified
- [ ] Full regression: all capture modes pass
- [ ] Emergency switch test: motors stop, X-ray off, no UART deadlock

---

## Phase 4: Safety Hardening (Week 6-7)

**Goal:** Activate all safety mechanisms for medical device compliance.

### Step 4.1: IWDG activation and testing
- **What:** `Safety_IWDG_Init()` is called in `main.c:99` but `Safety_IWDG_Kick()` must be called in the main loop. Verify:
  1. `Safety_IWDG_Kick()` is called in each main loop iteration (add to `Safety_CheckMainLoop()` or explicit call)
  2. Test watchdog reset recovery: inject an infinite loop, verify system resets within 1.36-3.76 seconds
  3. Test that `Safety_WasWatchdogReset()` reports correctly after reset
  4. Verify watchdog does NOT reset during long blocking operations (EEPROM write, CAN timeout waits)
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/safety.c`
  - `/home/issacs/work/projects/T2-Application/src/main.c`
- **Risk:** HIGH -- IWDG cannot be stopped once started. If main loop has legitimate long delays (e.g., `IntTimer_Delay(600)` at line 148), watchdog could trigger false resets.
- **Dependencies:** Phase 3 complete
- **Effort:** 10 hours
- **Verification:**
  - Controlled hang test: system resets within spec
  - Normal operation: 24-hour soak test with no watchdog resets
  - Post-reset: `Safety_WasWatchdogReset()` returns 1, error logged

### Step 4.2: Emergency stop full path test
- **What:** With `isr_new.c` active, test the complete emergency stop path:
  1. Press emergency switch during Pano capture (all motors running + X-ray on)
  2. Verify: all 8 motor timers disabled (< 500ns), X-ray off (< 2us), flag set
  3. Verify: main loop detects flag, logs via UART, system enters safe state
  4. Verify: release switch, system allows restart after explicit re-initialization
  5. Repeat for CT and Ceph capture modes
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/safety.c` (`Safety_EmergencyStop()`)
  - `/home/issacs/work/projects/T2-Application/src/isr_new.c` (`EXTI9_5_IRQHandler`)
  - `/home/issacs/work/projects/T2-Application/src/main.c` (`Safety_CheckMainLoop()`)
- **Risk:** HIGH -- this is the primary patient safety mechanism
- **Dependencies:** Step 4.1
- **Effort:** 8 hours
- **Verification:**
  - Logic analyzer on motor step pins + X-ray ready pin: all go inactive within 2us of EXTI edge
  - No UART output from ISR context (verify with breakpoint in printUart)
  - System restart after emergency release works correctly
  - 10 consecutive emergency stop cycles with no stuck state

### Step 4.3: X-ray interlock verification
- **What:** Test `Safety_VerifyXrayInterlock()`:
  1. When X-ray is OFF: readback confirms both pins inactive
  2. When X-ray is ON: readback confirms pins active (and matches commanded state)
  3. Inject fault: force GPIO mismatch, verify `SAFETY_ERR_XRAY_INTERLOCK` flag is set
  4. Verify main loop handles interlock fault (shuts down X-ray, logs error)
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/safety.c`
  - `/home/issacs/work/projects/T2-Application/src/tube.c`
- **Risk:** HIGH -- X-ray safety interlock is a regulatory requirement
- **Dependencies:** Step 4.1
- **Effort:** 6 hours
- **Verification:**
  - GPIO readback matches commanded state in 100/100 test cycles
  - Fault injection: system enters safe state within 1 main loop iteration

### Step 4.4: Fault injection testing
- **What:** Systematic fault injection for safety module:
  1. Stack overflow simulation (reduce stack size, trigger deep recursion) -- verify watchdog catches it
  2. Memory corruption: overwrite `g_safety` struct, verify recovery
  3. Timer peripheral failure: disable a motor timer mid-capture, verify error detection
  4. SPI bus lockup: hold TMC2660 CS low, verify timeout and error
  5. EEPROM read failure: disconnect I2C, verify graceful degradation
- **Files:** Multiple -- create test harness in `/home/issacs/work/projects/T2-Application/test/test_fault_injection.c`
- **Risk:** MEDIUM -- some faults require hardware modifications
- **Dependencies:** Steps 4.1-4.3
- **Effort:** 12 hours
- **Verification:** Each fault scenario results in defined, safe behavior (documented in test report)

### Phase 4 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 4.1 IWDG activation | 10 | HIGH |
| 4.2 Emergency stop full path | 8 | HIGH |
| 4.3 X-ray interlock verification | 6 | HIGH |
| 4.4 Fault injection testing | 12 | MEDIUM |
| **Total** | **36** | |

**Phase 4 Exit Criteria:**
- [ ] IWDG active in production build, 24-hour soak test passed
- [ ] Emergency stop verified in all 3 capture modes (motor + X-ray off < 2us)
- [ ] X-ray interlock readback verified, fault detection working
- [ ] Fault injection test report documented
- [ ] No unhandled fault condition identified

---

## Phase 5: Serial Migration (Week 7-8)

**Goal:** Replace monolithic `serial.c` with split modules.

### Step 5.1: Verify serial split module completeness
- **What:** Systematically verify that every function in old `serial.c` exists in one (and only one) of the new modules:
  - `uart_hw.c` -- UART peripheral init, byte send, debug print
  - `msg_protocol.c` -- 16-byte bracket protocol framing, message queue
  - `cmd_system.c` -- System commands (exposure, collimator, membrane)
  - `cmd_pano.c` -- Panoramic mode commands
  - `cmd_ct.c` -- CT/CBCT mode commands
  - `cmd_scan.c` -- Ceph scan mode commands
  - `cmd_geoalign.c` -- Geometry alignment commands
  - `cmd_geoalign_axes.c` -- Geometry alignment axis-specific commands
  - `cmd_geoalign_cal.c` -- Geometry alignment calibration
  - `cmd_eeprom_diag.c` -- EEPROM and diagnostic mode commands
  - `cmd_calibration.c` -- Sensor/collimator calibration commands
- **Why:** The 9,059-line `serial.c` was split into 11 files. Missing a function means broken functionality.
- **Files:** All listed above
- **Risk:** HIGH -- serial.c handles ALL user commands. Missing one command = broken workflow.
- **Dependencies:** Phase 3 complete (ISR migration includes UART ISRs)
- **Effort:** 12 hours
- **Verification:** `diff` function lists between old serial.c and new modules -- 100% coverage

### Step 5.2: Switch build from serial.c to split modules
- **What:**
  1. Exclude `serial.c` from build
  2. Ensure all split modules are in build (from Step 1.2)
  3. Fix any include/dependency issues
  4. Ensure `serial.h` still exports the public API needed by other modules
- **Files:**
  - `/home/issacs/work/projects/T2-Application/.cproject`
  - `/home/issacs/work/projects/T2-Application/inc/serial.h`
- **Risk:** HIGH
- **Dependencies:** Step 5.1
- **Effort:** 6 hours
- **Verification:** Build succeeds. Binary size should be similar (same code, just split files).

### Step 5.3: UART1 (PC application) command regression test
- **What:** Test every command category through the PC application:
  1. Mode selection: Pano, CT, Scan, Calibration, Geometry Align, EEPROM, Diagnostic, Reset
  2. Capture parameters: KV, mA, time, size, FOV
  3. Motor commands: vertical, frankfort, temple, column, canine
  4. Laser commands: on/off for all beam types
  5. Capture sequence: ready -> confirm -> capture -> complete
  6. Collimator: open/close, auto-align
  7. Arch selection: standard/child/sinus/TMJ
  8. Tube temperature readback
  9. Firmware version query
  10. Boot status messages
- **Files:** N/A (functional test via PC application)
- **Risk:** MEDIUM (regression testing)
- **Dependencies:** Step 5.2
- **Effort:** 16 hours
- **Verification:** Checklist of all UART commands with pass/fail per item

### Step 5.4: UART2 (Tablet) communication test
- **What:** Test tablet-specific messages:
  1. Membrane live signal (`[SP_MEMB_SIGN_]`)
  2. Boot done/fail messages
  3. Mode reset acknowledgment
  4. Capture cancel notification
  5. Membrane sound commands
  6. Membrane firmware version
- **Files:** N/A (functional test via tablet or UART2 terminal)
- **Risk:** LOW
- **Dependencies:** Step 5.3
- **Effort:** 4 hours
- **Verification:** All tablet messages sent/received correctly via UART2 terminal

### Phase 5 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 5.1 Serial split completeness audit | 12 | HIGH |
| 5.2 Build switch | 6 | HIGH |
| 5.3 PC UART1 command regression | 16 | MEDIUM |
| 5.4 Tablet UART2 test | 4 | LOW |
| **Total** | **38** | |

**Phase 5 Exit Criteria:**
- [ ] Old `serial.c` excluded from build
- [ ] All UART1 commands verified with PC application
- [ ] All UART2 commands verified with tablet or terminal
- [ ] No functional regression from serial split

---

## Phase 6: Code Quality (Week 8-10)

**Goal:** Clean up codebase for maintainability and medical device compliance.

### Step 6.1: Remove .bak files and dead code
- **What:** Delete all 16 `.bak` files:
  - `src/calibration.c.bak`, `src/ct_capture.c.bak`, `src/can.c.bak`
  - `inc/extern.h.bak`, `inc/motor.h.bak`, `inc/tube.h.bak`, `inc/version.h.bak`, `inc/can.h.bak`
  - `src/isr.c.bak`, `src/scan_capture.c.bak`, `src/motor.c.bak`, `src/main.c.bak`
  - `src/pano_capture.c.bak`, `src/misc1.c.bak`, `src/serial.c.bak`, `src/system.c.bak`
  
  Also remove: old `isr.c` (replaced by `isr_new.c`), old `serial.c` (replaced by split modules), commented-out code blocks, `#if 0` sections.
- **Files:** All listed above
- **Risk:** LOW (backups should be in git history, not file system)
- **Dependencies:** Phases 1-5 complete (all migrations done)
- **Effort:** 4 hours
- **Verification:** `find . -name '*.bak'` returns empty. `grep -rn '#if 0'` minimized.

### Step 6.2: Remove old motor.c (or reduce to config-only)
- **What:** After motor migration and ISR migration, `motor.c` should only contain:
  - Motor configuration constants (pin defines, pulley ratios, microstep settings)
  - Motor_Init_Check() and Motor_MoveInitPosition() (homing sequences)
  - Motor_Typedef struct definitions (still needed by capture modules via compat layer)
  Remove: Motor_GetNextRunCCR, Motor_GetNextOrgCCR, TMC2660_SPI_Init (now in tmc2660.c), duplicated SPI dispatch functions.
- **Files:** `/home/issacs/work/projects/T2-Application/src/motor.c`
- **Risk:** MEDIUM -- must preserve homing and config
- **Dependencies:** Phase 2 and Phase 3 complete
- **Effort:** 8 hours
- **Verification:** Build succeeds. motor.c reduced from ~4,912 to ~1,500 lines. All homing sequences work.

### Step 6.3: Add missing header guards and fix include hygiene
- **What:**
  1. Verify all `.h` files have proper `#ifndef __NAME_H__` / `#define` / `#endif` guards
  2. Remove unused includes (run `include-what-you-use` or manual audit)
  3. Replace `extern.h` "god header" usage: each `.c` file should include only the headers it actually needs
  4. Group includes: stdlib -> STM32 peripheral -> application (per coding-style.md)
- **Files:** All 30+ header files in `/home/issacs/work/projects/T2-Application/inc/`
- **Risk:** LOW -- structural cleanup only
- **Dependencies:** Step 6.1
- **Effort:** 8 hours
- **Verification:** Build succeeds. No header included unnecessarily (spot check 5 files).

### Step 6.4: Static analysis with cppcheck
- **What:** Run `cppcheck --enable=all --std=c99 --suppress=missingIncludeSystem -I inc/ -I Libraries/STM32F2xx_StdPeriph_Driver/inc/ -I Libraries/CMSIS/Device/ST/STM32F2xx/Include/ -I Libraries/CMSIS/Include/ src/` and fix:
  - All ERROR level findings
  - All WARNING level findings
  - STYLE findings selectively (focus on uninitialized variables, null pointer dereference, buffer overrun)
- **Files:** All `.c` files in `src/`
- **Risk:** LOW
- **Dependencies:** Step 6.3
- **Effort:** 12 hours
- **Verification:** cppcheck report shows zero ERROR and zero WARNING findings

### Step 6.5: MISRA-C:2012 compliance assessment
- **What:** Run PC-Lint or similar MISRA-C checker. Focus on:
  - Required rules (must fix for IEC 62304 Class B/C)
  - Advisory rules (document deviations)
  - Key areas: ISR shared variable access, pointer arithmetic, type conversions, implicit promotions
  - Document all deviations with rationale
- **Files:** All application source files
- **Risk:** LOW (analysis, not code change) -- but findings may require significant refactoring
- **Dependencies:** Step 6.4
- **Effort:** 20 hours (analysis + initial fixes)
- **Verification:** MISRA deviation report generated. All Required rule violations resolved or formally deviated.

### Step 6.6: Code documentation
- **What:**
  1. Add Doxygen-style comments to all public functions in new modules (motor_ctrl, safety, debug_log, tmc2660)
  2. Document all ISR handler entry/exit contracts
  3. Document SRAM/Flash memory map and budget
  4. Update ARCHITECTURE.md with post-migration state
  5. Create MODULE_INTERFACE.md documenting each module's public API
- **Files:**
  - All new module headers in `inc/`
  - `/home/issacs/work/projects/T2-Application/ARCHITECTURE.md` (update)
  - `/home/issacs/work/projects/T2-Application/MODULE_INTERFACE.md` (new)
- **Risk:** LOW
- **Dependencies:** All previous steps in Phase 6
- **Effort:** 12 hours
- **Verification:** Doxygen generates clean HTML output. No undocumented public function.

### Phase 6 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 6.1 Remove .bak files and dead code | 4 | LOW |
| 6.2 Reduce motor.c | 8 | MEDIUM |
| 6.3 Header guard / include hygiene | 8 | LOW |
| 6.4 cppcheck static analysis | 12 | LOW |
| 6.5 MISRA-C assessment | 20 | LOW |
| 6.6 Documentation | 12 | LOW |
| **Total** | **64** | |

**Phase 6 Exit Criteria:**
- [ ] Zero `.bak` files in repository
- [ ] `extern.h` no longer used as god header (or reduced to minimal shared types)
- [ ] cppcheck: zero errors, zero warnings
- [ ] MISRA-C deviation report complete
- [ ] ARCHITECTURE.md updated to reflect current state
- [ ] Full regression test passed after cleanup

---

## Phase 7: Testing Infrastructure (Week 10-12)

**Goal:** Establish automated testing for ongoing development.

### Step 7.1: Unit test expansion (target: 80% critical path coverage)
- **What:** Expand Unity test suite:
  - `test_motor_ctrl.c` -- profile builder edge cases (0 steps, max steps, overflow), position tracking, state machine transitions. Target: 90% coverage.
  - `test_tmc2660.c` -- register value construction, SPI sequence, current scaling. Target: 80%.
  - `test_safety.c` -- IWDG state machine, error flag operations, interlock logic. Target: 90%.
  - `test_debug_log.c` -- ring buffer full/empty/wrap, message truncation. Target: 95%.
  - `test_msg_protocol.c` -- bracket message parsing, malformed input, overflow. Target: 80%.
  - `test_shared_vars.c` -- volatile access patterns, alignment. Target: 60%.
- **Files:** `/home/issacs/work/projects/T2-Application/test/`
- **Risk:** LOW
- **Dependencies:** Phase 1 Step 1.6 (Unity framework), Phase 6 (code stabilized)
- **Effort:** 40 hours
- **Verification:** Coverage report shows targets met per module. All tests deterministic.

### Step 7.2: Hardware-in-the-loop (HIL) test jig design
- **What:** Design a test jig that can:
  1. Program the STM32F207 via SWD
  2. Simulate PC application (UART1 command injection)
  3. Simulate tablet (UART2 command injection)
  4. Monitor motor step pins (8 channels) via logic analyzer
  5. Monitor X-ray control pins
  6. Simulate emergency switch (relay on EXTI5)
  7. Simulate CAN bus responses (generator, collimator)
  8. Measure current consumption
  9. Controlled by a Raspberry Pi or similar running Robot Framework
- **Files:** Create `/home/issacs/work/projects/T2-Application/test/hil/` directory with test scripts
- **Risk:** MEDIUM (hardware design required)
- **Dependencies:** Phase 4 complete (safety module validated)
- **Effort:** 40 hours (design + initial setup)
- **Verification:** HIL jig can execute automated boot sequence test and emergency stop test

### Step 7.3: Regression test suite
- **What:** Create a documented regression test suite covering:
  1. Boot sequence (generator check, collimator check, motor check)
  2. Each capture mode entry/exit
  3. Emergency stop during each capture mode
  4. UART command set (automated via UART1 injection)
  5. CAN communication (automated via CAN bus simulator)
  6. Watchdog recovery
  7. EEPROM save/load
- **Files:** `/home/issacs/work/projects/T2-Application/test/regression/`
- **Risk:** LOW
- **Dependencies:** Step 7.2 (HIL jig for automated testing)
- **Effort:** 20 hours
- **Verification:** Regression suite runs end-to-end on HIL jig. Pass rate > 95% (remaining 5% = environmental factors).

### Step 7.4: CI pipeline (if applicable)
- **What:** If a CI server (Jenkins, GitHub Actions, GitLab CI) is available:
  1. On every push: host-side build (`make`), host-side unit tests (`make test`)
  2. Nightly: cppcheck, MISRA-C check, coverage report
  3. On release tag: full ARM build, binary size report, HIL trigger (if jig connected)
- **Files:** Create `/home/issacs/work/projects/T2-Application/.github/workflows/` or `Jenkinsfile`
- **Risk:** LOW
- **Dependencies:** Steps 7.1, 7.3
- **Effort:** 12 hours
- **Verification:** CI pipeline executes on push. Green badge for build + test.

### Phase 7 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 7.1 Unit test expansion | 40 | LOW |
| 7.2 HIL test jig design | 40 | MEDIUM |
| 7.3 Regression test suite | 20 | LOW |
| 7.4 CI pipeline | 12 | LOW |
| **Total** | **112** | |

**Phase 7 Exit Criteria:**
- [ ] Unit test coverage: motor_ctrl 90%, safety 90%, debug_log 95%, msg_protocol 80%
- [ ] HIL jig operational with automated boot + emergency stop tests
- [ ] Regression suite documented and executable
- [ ] CI pipeline running (if infrastructure available)

---

## Phase 8: Performance Optimization (Week 12-14)

**Goal:** Optimize resource usage after architecture is stable.

### Step 8.1: Profile buffer DMA mode evaluation
- **What:** Evaluate using DMA to transfer CCR values from profile buffer to timer ARR registers, eliminating ISR overhead entirely:
  1. STM32F207 DMA1/DMA2 channel availability (check if motor timer channels have DMA request mapping)
  2. DMA transfer: memory-to-peripheral, circular or normal mode
  3. Prototype for Motor_R (TIM8): DMA2 Stream 1, Channel 7 maps to TIM8_CH3
  4. Benchmark: compare ISR-based (~30 cycles) vs DMA (0 CPU cycles per step)
  5. Challenge: DMA cannot toggle GPIO -- step pulse generation still needs timer output compare mode, not manual GPIO toggle
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/motor_ctrl.c` (add DMA init)
  - `/home/issacs/work/projects/T2-Application/stm32_flash.ld` (may need DMA buffer section)
- **Risk:** MEDIUM -- DMA channel conflicts (check against all peripheral DMA usage), timer output compare mode change
- **Dependencies:** Phase 3 complete
- **Effort:** 20 hours
- **Verification:** Motor_R runs a Pano arch profile via DMA. CPU load measurement shows reduction. Step pulse timing matches ISR version.

### Step 8.2: Power consumption optimization
- **What:**
  1. Enter WFI (Wait For Interrupt) in main loop idle state (CAPTURE_CANCEL default case)
  2. Disable unused peripheral clocks (check RCC_AHBxPeriphClockCmd calls)
  3. Reduce motor hold current via TMC2660 when idle (already supported by `MotorCtrl_TMC2660_SetCurrent()`)
  4. Put TMC2660 into standby mode when motors idle for >30 seconds
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/main.c` (WFI in idle)
  - `/home/issacs/work/projects/T2-Application/src/motor_ctrl.c` (idle current reduction)
  - `/home/issacs/work/projects/T2-Application/src/tmc2660.c` (standby mode)
- **Risk:** LOW -- WFI exits on any interrupt, behavior unchanged
- **Dependencies:** Phase 4 complete (watchdog must handle WFI correctly)
- **Effort:** 8 hours
- **Verification:** Current measurement: idle state current reduced by measurable amount. Wake-up time < 100us.

### Step 8.3: Memory usage optimization
- **What:**
  1. Reduce `PROFILE_BUF_SIZE` from 4096 to actual maximum needed per motor (analyze arch trajectory tables: standard/child/sinus/TMJ LUT sizes)
  2. Share profile buffer between non-concurrent motors (e.g., Ceph motors C+S+A never run concurrently with Pano motors CNS+CWE+T)
  3. Move `const` motor configuration data to Flash explicitly (verify compiler places it in `.rodata`)
  4. Optimize `DbgLog_t` buffer size based on measured peak usage
  5. Report current SRAM usage via `arm-none-eabi-size` and map file analysis
- **Files:**
  - `/home/issacs/work/projects/T2-Application/inc/motor_ctrl.h` (PROFILE_BUF_SIZE)
  - `/home/issacs/work/projects/T2-Application/stm32_flash.ld` (verify sections)
- **Risk:** MEDIUM -- reducing buffer size could truncate long profiles
- **Dependencies:** Phase 2 complete (profile data available)
- **Effort:** 12 hours
- **Verification:** SRAM usage < 80KB (leaving 32KB headroom). All capture modes still work with reduced buffers.

### Step 8.4: Boot time optimization
- **What:**
  1. Profile boot sequence (from Reset_Handler to `[SP_BOOT_DONE_]` message)
  2. Identify slow operations: EEPROM load, generator CAN init (timeout-based), motor init check
  3. Parallelize where possible: start CAN init while loading EEPROM
  4. Optimize TMC2660 SPI init sequence (8 motors x 4 registers = 32 SPI transactions)
  5. Add boot time measurement to debug output
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/main.c`
  - `/home/issacs/work/projects/T2-Application/src/system.c`
- **Risk:** LOW
- **Dependencies:** Phase 6 complete
- **Effort:** 8 hours
- **Verification:** Boot time measurement before/after. Target: < 3 seconds to `[SP_BOOT_DONE_]`.

### Phase 8 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 8.1 DMA mode evaluation | 20 | MEDIUM |
| 8.2 Power consumption optimization | 8 | LOW |
| 8.3 Memory usage optimization | 12 | MEDIUM |
| 8.4 Boot time optimization | 8 | LOW |
| **Total** | **48** | |

**Phase 8 Exit Criteria:**
- [ ] DMA feasibility assessment documented (go/no-go decision)
- [ ] SRAM usage < 80KB documented
- [ ] Boot time measured and optimized
- [ ] Power consumption measured in idle and active states

---

## Phase 9: Future Architecture (Week 14+)

**Goal:** Long-term technology evolution path.

### Step 9.1: i.MX8MP integration path (CAN/UART bridge to Linux)
- **What:** Design the communication bridge between STM32F207 (motor/safety controller) and i.MX8MP (image processing / UI):
  1. Define CAN message protocol for motor commands (i.MX8MP -> STM32) and status (STM32 -> i.MX8MP)
  2. Alternative: UART protocol with defined message format (current bracket protocol adapted)
  3. Design responsibility split: STM32 retains real-time motor control + safety; i.MX8MP handles imaging, UI, networking
  4. Create interface specification document
- **Files:** Create `/home/issacs/work/projects/T2-Application/docs/imx8_interface_spec.md`
- **Risk:** LOW (design phase, no code changes)
- **Dependencies:** Phase 6 (clean architecture)
- **Effort:** 24 hours
- **Verification:** Interface specification reviewed and approved by hardware and software teams

### Step 9.2: RTOS evaluation (FreeRTOS migration feasibility)
- **What:** Assess FreeRTOS migration for:
  1. SRAM budget: FreeRTOS kernel ~6-10KB Flash, ~1KB RAM per task. With 112KB SRAM and ~66KB for motor profiles, only ~36KB remains. FreeRTOS may require buffer size reduction.
  2. Task decomposition: motor control task, serial processing task, CAN processing task, safety monitor task, idle task
  3. Priority mapping: motor ISRs remain ISR-level (not tasks). Tasks handle main-loop work.
  4. Benefits: deterministic serial processing, CAN processing without polling, sleep in idle task
  5. Risks: increased complexity, harder debugging, certification impact (IEC 62304)
  6. Recommendation: defer until i.MX8MP integration changes the STM32's role
- **Files:** Create `/home/issacs/work/projects/T2-Application/docs/rtos_evaluation.md`
- **Risk:** LOW (analysis only)
- **Dependencies:** Step 8.3 (memory budget known)
- **Effort:** 16 hours
- **Verification:** Evaluation document with go/no-go recommendation

### Step 9.3: OTA firmware update capability
- **What:** Design OTA update path:
  1. Current bootloader at 0x08000000-0x0801FFFF (128KB) -- verify it supports dual-bank or A/B update
  2. Add CRC32 verification for firmware image before flash write
  3. Define update transport: UART from PC application, or CAN from i.MX8MP
  4. Implement rollback: if new firmware fails boot (watchdog reset 3 times), revert to backup
  5. Secure boot: sign firmware image with ECDSA, verify in bootloader
- **Files:**
  - `/home/issacs/work/projects/T2-Application/src/boot.c` (extend bootloader)
  - `/home/issacs/work/projects/T2-Application/docs/ota_design.md` (new)
- **Risk:** HIGH (bootloader changes can brick device)
- **Dependencies:** Phase 4 (watchdog), Phase 8 (boot time)
- **Effort:** 40 hours (design + implementation)
- **Verification:** OTA update from known-good to test firmware, then rollback. 100% success rate over 20 cycles.

### Step 9.4: Remote diagnostics
- **What:** Design diagnostic data collection:
  1. Periodic health report: motor position, temperature, error flags, capture count
  2. Error log: persistent storage of last N safety events (EEPROM or reserved flash sector)
  3. Transport: via i.MX8MP ethernet/WiFi to cloud, or via UART to PC application
  4. Privacy: no patient data in diagnostic stream
- **Files:** Create `/home/issacs/work/projects/T2-Application/docs/remote_diagnostics.md`
- **Risk:** LOW (design phase)
- **Dependencies:** Step 9.1 (i.MX8MP interface)
- **Effort:** 12 hours
- **Verification:** Design document reviewed

### Phase 9 Summary
| Item | Effort (hrs) | Risk |
|------|-------------|------|
| 9.1 i.MX8MP interface design | 24 | LOW |
| 9.2 RTOS evaluation | 16 | LOW |
| 9.3 OTA firmware update | 40 | HIGH |
| 9.4 Remote diagnostics design | 12 | LOW |
| **Total** | **92** | |

---

## Global Risk Register

| Risk | Likelihood | Impact | Phase | Mitigation |
|------|-----------|--------|-------|------------|
| SRAM overflow (motor profile buffers + old Motor_Typedef coexist) | HIGH | HIGH | 2 | Reduce PROFILE_BUF_SIZE during migration; monitor with `arm-none-eabi-size` after every step |
| Duplicate ISR handler symbols (isr.c + isr_new.c + emergency.c) | HIGH | HIGH | 3 | Use `#ifdef USE_ISR_NEW` guards; never link both ISR files |
| Patient collision from motor profile divergence | MEDIUM | CRITICAL | 2 | Profile equivalence test (Step 2.1) before any hardware test; oscilloscope verification |
| Unintended X-ray exposure from tube.c init bug | MEDIUM | CRITICAL | 1 | Fix Tube.bXrayOnOff=FALSE immediately (Step 1.5); X-ray interlock verification (Phase 4) |
| Watchdog false reset during blocking EEPROM/CAN operations | MEDIUM | HIGH | 4 | Audit all blocking waits; add IWDG kick in long loops; 24-hour soak test |
| Serial split missing a command handler | MEDIUM | MEDIUM | 5 | Systematic function audit (Step 5.1) before build switch |
| Flash corruption from boot.c sizeof bug | LOW | CRITICAL | 1 | Fix immediately (Step 1.4); verify with read-back after write |
| CAN message injection via open filter | HIGH | HIGH | 1 | Fix CAN filter mask immediately (Step 1.5) |

---

## Effort Summary

| Phase | Description | Weeks | Person-Hours |
|-------|-------------|-------|-------------|
| 1 | Stabilization | 1-2 | 54 |
| 2 | Motor Migration | 3-5 | 70 |
| 3 | ISR Migration | 5-6 | 30 |
| 4 | Safety Hardening | 6-7 | 36 |
| 5 | Serial Migration | 7-8 | 38 |
| 6 | Code Quality | 8-10 | 64 |
| 7 | Testing Infrastructure | 10-12 | 112 |
| 8 | Performance Optimization | 12-14 | 48 |
| 9 | Future Architecture | 14+ | 92 |
| **Total** | | **~14 weeks** | **544** |

**Note:** Phases 1-5 (stabilization through serial migration) total 228 person-hours and represent the minimum viable migration. Phases 6-9 are value-add improvements. At 1 engineer full-time, Phases 1-5 require approximately 6 weeks of focused work; Phases 6-9 add another 8 weeks.

---

## Key File Paths Referenced

**New architecture (created, integration pending):**
- `/home/issacs/work/projects/T2-Application/src/motor_ctrl.c`
- `/home/issacs/work/projects/T2-Application/inc/motor_ctrl.h`
- `/home/issacs/work/projects/T2-Application/src/tmc2660.c`
- `/home/issacs/work/projects/T2-Application/inc/tmc2660.h`
- `/home/issacs/work/projects/T2-Application/src/isr_new.c`
- `/home/issacs/work/projects/T2-Application/inc/isr_new.h`
- `/home/issacs/work/projects/T2-Application/src/safety.c`
- `/home/issacs/work/projects/T2-Application/inc/safety.h`
- `/home/issacs/work/projects/T2-Application/src/debug_log.c`
- `/home/issacs/work/projects/T2-Application/inc/debug_log.h`
- `/home/issacs/work/projects/T2-Application/inc/shared_vars.h`
- `/home/issacs/work/projects/T2-Application/src/motor_compat.c`
- `/home/issacs/work/projects/T2-Application/inc/motor_compat.h`
- `/home/issacs/work/projects/T2-Application/src/uart_hw.c`
- `/home/issacs/work/projects/T2-Application/inc/uart_hw.h`
- `/home/issacs/work/projects/T2-Application/src/msg_protocol.c`
- `/home/issacs/work/projects/T2-Application/inc/msg_protocol.h`
- `/home/issacs/work/projects/T2-Application/src/cmd_system.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_pano.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_ct.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_scan.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_geoalign.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_geoalign_axes.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_geoalign_cal.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_eeprom_diag.c`
- `/home/issacs/work/projects/T2-Application/src/cmd_calibration.c`
- `/home/issacs/work/projects/T2-Application/inc/cmd_dispatch.h`

**Old architecture (to be replaced/reduced):**
- `/home/issacs/work/projects/T2-Application/src/motor.c`
- `/home/issacs/work/projects/T2-Application/src/isr.c`
- `/home/issacs/work/projects/T2-Application/src/serial.c`
- `/home/issacs/work/projects/T2-Application/src/emergency.c`
- `/home/issacs/work/projects/T2-Application/inc/extern.h`

**Build and config:**
- `/home/issacs/work/projects/T2-Application/.cproject`
- `/home/issacs/work/projects/T2-Application/stm32_flash.ld`

**Documentation:**
- `/home/issacs/work/projects/T2-Application/ARCHITECTURE.md`
- `/home/issacs/work/projects/T2-Application/CODE_REVIEW.md`