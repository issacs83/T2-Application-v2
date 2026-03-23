/*
 * safety_monitor.c : Safety monitor orchestration (App layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * TODO: This module is scaffolding for the target architecture migration.
 *       Provides App-layer wrappers around safety.c functions. Currently
 *       not called from the main loop (App_Run calls Safety_CheckMainLoop
 *       directly). Will be integrated when state machine migration is complete.
 *       Do NOT delete — represents the target architecture.
 *
 * Memory: ~200 bytes Flash, 0 bytes SRAM (uses safety.c state).
 */

#include "safety_monitor.h"
#include "safety.h"
#include "debug_log.h"

/* ------------------------------------------------------------------ */
void SafetyMonitor_Init(void)
{
    Safety_IWDG_Init();
    DbgLog_Init();
}

/* ------------------------------------------------------------------ */
void SafetyMonitor_Poll(void)
{
    Safety_CheckMainLoop();
}

/* ------------------------------------------------------------------ */
bool SafetyMonitor_IsEmergencyActive(void)
{
    return Safety_GetEmergencyFlag() != 0;
}

/* ------------------------------------------------------------------ */
void SafetyMonitor_ClearEmergency(void)
{
    Safety_ClearEmergencyFlag();
}

/* ------------------------------------------------------------------ */
bool SafetyMonitor_WasWatchdogReset(void)
{
    return Safety_WasWatchdogReset() != 0;
}

/* ------------------------------------------------------------------ */
uint8_t SafetyMonitor_GetErrors(void)
{
    return Safety_GetErrorFlags();
}

/* ------------------------------------------------------------------ */
void SafetyMonitor_ClearErrors(uint8_t flags)
{
    Safety_ClearErrorFlags(flags);
}
