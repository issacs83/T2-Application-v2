/*
 * app_main.h : Application entry point (App_Init + App_Run)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Top-level application orchestration. Called from Core/Src/main.c.
 *
 * App_Init() performs:
 *   1. System peripheral initialization (System_Init)
 *   2. Port layer initialization (serial, CAN, motor, sensors, GPIO)
 *   3. Domain module initialization (debug log)
 *   4. Safety subsystem initialization (IWDG, emergency)
 *   5. EEPROM configuration loading
 *   6. Generator + Collimator + Motor boot checks
 *
 * App_Run() is the main super-loop:
 *   1. Dispatch to current capture mode handler
 *   2. Process CAN messages
 *   3. Safety checks + watchdog kick
 *   4. Debug log flush
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
 */
void App_Run(void);

#endif /* APP_MAIN_H */
