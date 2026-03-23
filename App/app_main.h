/*
 * app_main.h : Application entry point (App_Init + App_Run)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Top-level application orchestration. Called from Core/Src/main.c.
 *
 * App_Init() performs:
 *   1. System peripheral initialization (System_Init)
 *   2. Debug log + motor controller + safety init
 *   3. EEPROM configuration loading
 *   4. Generator + Collimator + Motor boot checks
 *   5. State machine initialization
 *
 * App_Run() is the non-blocking main super-loop:
 *   1. Safety_CheckMainLoop (watchdog + emergency)
 *   2. CAN message processing
 *   3. Mode change detection (CurCaptureMode -> SM events)
 *   4. StateMachine_Run (non-blocking dispatch)
 *   5. DbgLog_Flush
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

/**
 * Initialize all application subsystems.
 * Call once from main() before entering the super-loop.
 */
void App_Init(void);

/**
 * Main application super-loop. Never returns.
 * Every function called from App_Run() is non-blocking.
 */
void App_Run(void);

#endif /* APP_MAIN_H */
