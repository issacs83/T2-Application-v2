# T2-Application Code Review (2026-03-23)

## Project: STM32F2xx Medical X-ray Device Firmware
## Verdict: REQUEST_CHANGES

---

## CRITICAL (6건)

| # | File:Line | Issue | Fix |
|---|-----------|-------|-----|
| 1 | extern.h:56 | `g_bTiltStatus` defined as non-`extern` in header → each .c gets own copy → tilt safety interlock broken | Change to `extern bool g_bTiltStatus;` in header, define in one .c file |
| 2 | system.h:116, main.c:68 | `CurCaptureMode` lacks `volatile` → ISR writes missed by main loop → mode cancel during X-ray exposure missed | Add `volatile` qualifier |
| 3 | isr.c:782 | `Sensor_P.CaptureCcr[ArrayIndex]` unbounded array access → Hard Fault during pano capture | Add bounds check before access |
| 4 | serial.c:1088-1141 | UART buffer overflow: `buffer.Data[16]` with no count check in default case | Add `if (count < UART_MSG_SIZE - 1)` guard |
| 5 | emergency.c:167 | `printUart()` called from emergency ISR → deadlock/stack overflow in safety-critical path | Set flag in ISR, log in main loop |
| 6 | boot.c:100 | Flash write loop uses sizeof as word count but iterates by bytes → 4x overwrite → flash corruption | Change to `sizeof(Bootloader_DataTypeDef) / sizeof(uint32_t)` |

## HIGH (6건)

| # | File:Line | Issue | Fix |
|---|-----------|-------|-----|
| 7 | isr.c (all motor ISRs) | ISRs excessively long with complex function calls → priority inversion risk | Set flags in ISR, process in main loop |
| 8 | isr.c:351 | `printUart()` in CAN RX ISR → stalls CAN processing during capture | Set error flag, log in main loop |
| 9 | timer.h:26-29 | `ms_To_CCR` macro precedence bug, `msec_To_sec = 1/1000` = integer 0 | Wrap params, use `1.0/1000.0` |
| 10 | can.c:253 | CAN filter accepts ALL messages (mask 0x0000) → arbitrary message injection | Set proper filter masks |
| 11 | motor.c:1491,1644 | TMC2660 array access without bounds check | Add bounds guards |
| 12 | tube.c:168 | `Tube.bXrayOnOff` initialized TRUE → unintended X-ray possible | Init to FALSE, require explicit enable |

## MEDIUM — Code Quality

- serial.c: 9,960 lines → split into uart_config, uart_parse, uart_cmd_*
- motor.c: 5,613 lines → split into motor_config, motor_spi, motor_motion
- 8 motor ISRs with duplicated code → extract common handler
- 3 SPI dispatch functions nearly identical → lookup table
- Magic numbers throughout ISRs → named constants
- .bak files and commented-out dead code → remove
- No unit tests → add Unity/CppUTest with HAL mocking
- extern.h "god header" → split into module-specific headers
- ISR-shared variables missing volatile (nHallSensorCount, bColumnStop, etc.)

## Positive Observations

- CAN TX validates KV/mA ranges before sending
- Emergency stop hardware interrupt at highest priority
- Motor origin-seek includes step count safety limits
- Bootloader CRC verification before app jump
- Consistent checkPreCaptureStatus() pattern before capture
