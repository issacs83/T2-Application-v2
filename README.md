# T2-Application v2

STM32F207 dental X-ray imaging device firmware — redesigned with 3-layer architecture.

## Architecture

```
Core/Src/main.c  →  App_Init() + App_Run()
        │
        ▼
┌─────────────────────────────────────────┐
│  App Layer (Business Logic)             │
│  app_main, state_machine, safety_monitor│
│  pano/ct/scan_sequence, calibration     │
├─────────────────────────────────────────┤
│  Domain Layer (HW-Independent)          │
│  motor_profile, motor_controller        │
│  packet_parser, cmd_handler, ring_buffer│
│  arch_tables                            │
├─────────────────────────────────────────┤
│  Port Layer (HAL Wrapper)               │
│  hw_serial, hw_can, hw_motor            │
│  hw_tmc2660, hw_sensor, hw_tube         │
│  hw_eeprom, hw_gpio, hw_isr            │
└─────────────────────────────────────────┘
        │
        ▼
    Libraries/ (STM32F2xx StdPeriph)
```

## Key Design Principles

- **Domain = ZERO hardware dependency** (no STM32 includes)
- **CommInterface_t** function pointer vtable for UART/CAN abstraction
- **MotorInterface_t** for motor control abstraction
- **ISR < 30 cycles** via pre-computed LUT profiles (Q16.16 fixed-point)
- **IWDG watchdog**, ISR-safe emergency stop, X-ray interlock readback

## Directory Structure

| Directory | Description |
|-----------|-------------|
| `App/` | Application layer — business logic, capture sequences |
| `Domain/` | Domain layer — pure algorithms, no HW dependency |
| `Port/` | Port layer — hardware abstraction (StdPeriph wrapper) |
| `Core/Src/` | Entry point (main.c → App_Init + App_Run) |
| `Libraries/` | STM32F2xx StdPeriph Driver + CMSIS |
| `Legacy/` | Original src/inc files (reference only) |

## Hardware

- MCU: STM32F207VGTx (120MHz Cortex-M3, 1MB Flash, 128KB SRAM)
- 8 stepper motors (TMC2660 drivers, SPI bit-bang)
- CAN2 bus (generator + collimator subsystems)
- UART1 (PC, 115200) + UART2 (Tablet, 19200)
- X-ray tube control (GPIO)

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — Full system analysis
- [CODE_REVIEW.md](CODE_REVIEW.md) — Code review findings
- [ROADMAP.md](ROADMAP.md) — 9-phase improvement roadmap

## Origin

Redesigned from [T2-Application](https://github.com/issacs83/T2-Application) (original monolithic firmware).
