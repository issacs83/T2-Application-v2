/*
 * app_main.c : Application entry point implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * This file wires together Port -> Domain -> App layers.
 * It is the only App-layer file that directly references both
 * Port and legacy src/ modules.
 *
 * App_Run() is the non-blocking main loop. Every function called
 * from App_Run() must return within ~100us. No blocking waits.
 *
 * Memory: ~1400 bytes Flash, ~20 bytes SRAM.
 */

/* System / legacy headers */
#include "stm32f2xx.h"
#include "extern.h"
#include "system.h"
#include "configuration.h"

#ifdef USE_I2C_EEPROM
#include "eeprom.h"
#endif

#include "can.h"
#include "serial.h"
#include "motor.h"
#include "timer.h"
#include "misc1.h"
#include "Emergency.h"
#include "diagnostic.h"

/* CAN_Collimator_SendMessage is defined in can.c but not declared in can.h */
extern bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value,
                                         uint16_t debug, uint32_t timeOut,
                                         uint8_t retryCnt);

/* New architecture headers */
#include "app_main.h"
#include "state_machine.h"
#include "safety_monitor.h"
#include "ct_sequence.h"
#include "pano_sequence.h"
#include "scan_sequence.h"
#include "calibration.h"

/* Port layer */
#include "../Port/hw_serial.h"
#include "../Port/hw_can.h"
#include "../Port/hw_motor.h"
#include "../Port/hw_gpio.h"
#include "../Port/hw_sensor.h"
#include "../Port/hw_tube.h"
#include "../Port/hw_eeprom.h"

/* Domain layer */
#include "../Domain/ring_buffer.h"

/* Legacy module headers for boot sequence */
#include "shared_vars.h"
#include "safety.h"
#include "debug_log.h"
#include "motor_ctrl.h"
#include "isr_new.h"

/* ================================================================== */
/* Private variables                                                  */
/* ================================================================== */

/* Boot status flags - accessed by state_machine.c for idle LED */
bool g_boot_gen_ok;
bool g_boot_col_ok;
bool g_boot_mot_ok;

#define UART_MEMBRANELIVE_SIGNAL_SEND   "[SP_MEMB_SIGN_]"

/* ================================================================== */
/* CAN message processing (non-blocking)                               */
/* ================================================================== */
static void app_process_can(void)
{
    if (g_can_rx_flag) {
        __disable_irq();
        CAN_RxMessage = g_can_rx_msg;
        g_can_rx_flag = 0;
        __enable_irq();
        CAN_ParseMessage();
    }
}

/* ================================================================== */
/* Mode change detection                                               */
/* Bridges legacy CurCaptureMode writes (from UART command parser)     */
/* into state machine events.                                          */
/* ================================================================== */
static volatile OpStatus_t g_prev_capture_mode = CAPTURE_CANCEL;

static void app_check_mode_change(void)
{
    OpStatus_t cur = CurCaptureMode;

    if (cur == g_prev_capture_mode) {
        return;
    }

    /* Only post mode events when state machine is idle */
    if (StateMachine_GetState() != ST_IDLE) {
        /* If a cancel was requested, post cancel event */
        if (cur == CAPTURE_CANCEL || cur == RESET_MODE) {
            StateMachine_PostEvent(EVT_CANCEL);
        }
        g_prev_capture_mode = cur;
        return;
    }

    switch (cur) {
    case CAPTURE_PANO:      StateMachine_PostEvent(EVT_MODE_PANO);        break;
    case CAPTURE_CT:        StateMachine_PostEvent(EVT_MODE_CT);          break;
    case CAPTURE_SCAN:      StateMachine_PostEvent(EVT_MODE_SCAN);        break;
    case CALIBRATION_MODE:  StateMachine_PostEvent(EVT_MODE_CALIBRATION); break;
    case GEOMETRY_ALIGN_MODE: StateMachine_PostEvent(EVT_MODE_GEO_ALIGN); break;
    case EEPROM_MODE:       StateMachine_PostEvent(EVT_MODE_EEPROM);      break;
    case DIAGNOSTIC_MODE:   StateMachine_PostEvent(EVT_MODE_DIAGNOSTIC);  break;
    case RESET_MODE:        StateMachine_PostEvent(EVT_MODE_RESET);       break;
    case CAPTURE_CANCEL:
    default:
        break;
    }

    g_prev_capture_mode = cur;
}

/* ================================================================== */
/* App_Init                                                           */
/* ================================================================== */

void App_Init(void)
{
    /*
     * Step 1: Core system initialization.
     * System_Init() configures clocks, GPIO, UART, CAN, timers,
     * sensors, tube, EEPROM, emergency switch, motors.
     */
    System_Init();

    /*
     * Step 2: Initialize new architecture modules.
     */
    DbgLog_Init();
    MotorCtrl_Init();
    Safety_IWDG_Init();

    printUart(DBG_MSG_PC, "System configuration is complete");

    /*
     * Step 3: Load EEPROM configuration.
     */
    if ((sysInfo.nFW_Com_Value & 0x0001) == 0x0000) {
        EEPRom_Load_Apply_minus_Offset();
    } else {
        EEPRom_Load_Align_Offset();
    }

    /*
     * Step 4: Boot-time subsystem checks.
     * Only proceed if emergency switch is not pressed.
     */
    if (EMG_ProcessSwitch()) {
        printUart(DBG_MSG_PC, "=============Generator Check=============");
        g_boot_gen_ok = SysInitGenerator();
        if (!g_boot_gen_ok) {
            printUart(DBG_MSG_PC, "Boot Condition : Generator error");
        }
        printUart(DBG_MSG_PC, "======================================");

        printUart(DBG_MSG_PC, "=============Collimator Check=============");
        g_boot_col_ok = SysInitCollimator();
        if (!g_boot_col_ok) {
            printUart(DBG_MSG_PC, "Boot Condition : Collimator error");
        }
        printUart(DBG_MSG_PC, "======================================");

        g_boot_mot_ok = Motor_Init_Check();
        if (!g_boot_mot_ok) {
            printUart(DBG_MSG_PC, "Boot Condition : Motor error");
        }
    } else {
        g_boot_gen_ok = false;
        g_boot_col_ok = false;
        g_boot_mot_ok = false;
    }

    /*
     * Step 5: Report boot status.
     */
    if (g_boot_gen_ok && g_boot_col_ok && g_boot_mot_ok) {
        printUart(DBG_MSG_PC, "[SP_BOOT_DONE_]");
        gMachStat.bBootDone = SET;
    } else {
        printUart(DBG_MSG_PC, "[SP_BOOT_FAIL_]");
        gMachStat.bBootErr = SET;
    }

    /*
     * Step 6: Send membrane live signal to tablet.
     */
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, UART_MEMBRANELIVE_SIGNAL_SEND);
#endif

    /* Short delay for boot completion */
    IntTimer_Delay(600);
    while (!IntTimer_GetStatus()) {
        /* wait - acceptable during boot only */
    }

    /* Clear first-capture flags */
    CtParam.bFirstCT = RESET;
    PanoParam.bFirstPano = RESET;

    /* Initialize membrane data */
    UART_Memb_Init_Data();

    /* Initialize state machine */
    StateMachine_Init();

    /* Sync mode tracking */
    g_prev_capture_mode = CurCaptureMode;
}

/* ================================================================== */
/* App_Run : Non-blocking main loop                                    */
/* ================================================================== */

void App_Run(void)
{
    while (1) {
        /*
         * 1. Safety: watchdog kick + emergency check.
         *    Must be first to ensure watchdog is fed even if
         *    other subsystems are slow.
         */
        Safety_CheckMainLoop();

        /*
         * 2. CAN message processing.
         *    Copies ISR-received message and parses it.
         */
        app_process_can();

        /*
         * 3. Mode change detection.
         *    Bridges legacy CurCaptureMode writes into SM events.
         */
        app_check_mode_change();

        /*
         * 4. State machine dispatch.
         *    Processes one event and runs the active state handler.
         *    Every handler returns within ~100us.
         */
        StateMachine_Run();

        /*
         * 5. Debug log flush.
         *    Sends buffered debug messages to UART.
         */
        DbgLog_Flush();
    }
}
