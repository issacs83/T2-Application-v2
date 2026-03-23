# T2-Application Architecture Analysis (2026-03-23)

## Summary
STM32F207 (120MHz Cortex-M3) dental X-ray imaging firmware (Pano/CBCT/Ceph)
~26,000 lines of application code, 8 stepper motors, CAN bus, dual UART

## Module Map

| Module | Lines | Responsibility |
|--------|-------|----------------|
| serial.c | 9,059 | UART protocol, ALL command dispatch |
| motor.c | 4,912 | 8 motor config, profiles, TMC2660 SPI |
| eeprom.c | 1,383 | I2C EEPROM, alignment offsets |
| timer.c | 1,136 | Timer init, delay, elapsed |
| can.c | 1,278 | CAN2 bus (generator/collimator) |
| misc1.c | 1,152 | Column, exposure switch, lasers |
| ct_capture.c | 918 | CBCT capture orchestration |
| pano_capture.c | 795 | Panoramic capture orchestration |
| isr.c | 778 | ALL interrupt handlers |
| scan_capture.c | 670 | Ceph scan capture |
| sensor.c | 451 | Pano/Scan/CT sensors |
| main.c | 323 | Entry point, super-loop |
| system.c | 302 | Init, health checks |
| arch.c | 305 | Panoramic arch trajectory LUTs |
| tube.c | 244 | X-ray tube control |
| boot.c | 206 | Bootloader, flash ops |
| calibration.c | 162 | Sensor/collimator calibration |
| emergency.c | 148 | Emergency switch (EXTI5) |

## Timer Allocation

| Timer | Motor/Function | Priority |
|-------|---------------|----------|
| TIM1 | Motor V (Vertical) | 0/0 |
| TIM2 | Motor C (Ceph detector) | 0/0 |
| TIM3 | Motor CNS (Chinrest V) | 0/0 |
| TIM4 | Hall sensor encoder | 3/1 |
| TIM5 | Motor T (Temple) | 0/0 |
| TIM6 | IntTimer (1ms delay) | 3/0 |
| TIM7 | ElapsedTimer (ms counter) | 3/0 |
| TIM8 | Motor R (Rotation) | 0/0 |
| TIM9 | Motor A (Gantry) | 0/0 |
| TIM10 | Motor S (2nd collimator) | 0/0 |
| TIM11 | Motor CWE (Chinrest H) | 0/0 |
| TIM12 | Pano/CT sensor sync | -- |
| TIM14 | X-ray tube PPS | -- |

## Capture Modes

### Panoramic
- Motors: R(rotation), V(vertical), CNS, CWE, T
- Arch trajectory from Flash LUT (standard/child/sinus/TMJ)
- Sensor sync: TIM12, X-ray PPS: TIM14

### CT/CBCT
- Motors: R(365° rotation), V, CNS, CWE
- FOV: 5x5 to 15x15 (stitch mode for 15x15)
- 400 or 600 frames

### Cephalometric Scan
- Motors: R(90° to ceph position), A(gantry), C(detector), S(slit)
- Synchronized C+S scan
- Modes: Full_Lateral, Lateral, Carpus, PA, AP, Waters, SVM, EarLod

## Critical Issues
1. Float in ISR (Cortex-M3 no FPU) → 2-10us per ISR
2. No volatile on ISR-shared variables
3. No hardware watchdog (IWDG)
4. g_bTiltStatus multiply defined
5. printUart() from ISR context
6. serial.c 9K lines, motor.c 5K lines
7. No unit tests
8. CAN filter accepts all messages
