/*
 * safety_monitor.h : IWDG, emergency, X-ray interlock (App layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * App-layer orchestration of safety functions.
 * Calls into Port/safety.c for hardware operations and
 * Port/hw_tube.c for X-ray interlock verification.
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize all safety subsystems (IWDG, emergency GPIO).
 * Call once at startup after all peripherals are initialized.
 */
void SafetyMonitor_Init(void);

/**
 * Main-loop safety check. Must be called every iteration.
 * Actions:
 *   1. Kick watchdog
 *   2. Check emergency flag, handle deferred UART logging
 *   3. Verify X-ray interlock state
 *   4. Flush debug log ring buffer
 */
void SafetyMonitor_Poll(void);

/**
 * Get emergency stop active status.
 */
bool SafetyMonitor_IsEmergencyActive(void);

/**
 * Clear emergency state after operator has resolved the condition.
 */
void SafetyMonitor_ClearEmergency(void);

/**
 * Check if last reset was caused by watchdog timeout.
 */
bool SafetyMonitor_WasWatchdogReset(void);

/**
 * Get error flags bitmask.
 */
uint8_t SafetyMonitor_GetErrors(void);

/**
 * Clear specific error flags.
 */
void SafetyMonitor_ClearErrors(uint8_t flags);

#endif /* SAFETY_MONITOR_H */
