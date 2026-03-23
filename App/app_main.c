/*
 * app_main.c : Application entry point implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * This file wires together Port -> Domain -> App layers.
 * It is the only App-layer file that directly references both
 * Port and legacy src/ modules.
 *
 * Memory: ~1200 bytes Flash, ~16 bytes SRAM.
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

static bool g_boot_gen_ok;
static bool g_boot_col_ok;
static bool g_boot_mot_ok;

#define UART_MEMBRANELIVE_SIGNAL_SEND   "[SP_MEMB_SIGN_]"

/* ================================================================== */
/* App_Init                                                           */
/* ================================================================== */

void App_Init(void)
{
    /*
     * Step 1: Core system initialization.
     * System_Init() configures clocks, GPIO, UART, CAN, timers,
     * sensors, tube, EEPROM, emergency switch, motors.
     * This is the existing monolithic init from system.c.
     */
    System_Init();

    /*
     * Step 2: Initialize new architecture modules.
     * These must come after System_Init() because they depend on
     * peripheral clocks and GPIO being configured.
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
        /* wait */
    }

    /* Clear first-capture flags */
    CtParam.bFirstCT = RESET;
    PanoParam.bFirstPano = RESET;

    /* Initialize membrane data */
    UART_Memb_Init_Data();

    /* Initialize state machine */
    StateMachine_Init();
}

/* ================================================================== */
/* App_Run                                                            */
/* ================================================================== */

void App_Run(void)
{
    while (1) {
        /*
         * Main dispatch: route to the active capture/mode handler.
         * Uses the legacy CurCaptureMode global for backward
         * compatibility with existing capture modules.
         */
        switch (CurCaptureMode) {
        case CAPTURE_PANO:
            PanoSequence_Run();
            break;

        case CAPTURE_CT:
            CtSequence_Run();
            break;

        case CAPTURE_SCAN:
            ScanSequence_Run();
            break;

        case CALIBRATION_MODE:
            AppCalibration_Run();
            break;

        case GEOMETRY_ALIGN_MODE:
            AppGeoAlign_Run();
            break;

#ifdef USE_I2C_EEPROM
        case EEPROM_MODE:
            EEPRom_ModeLoop();
            break;
#endif

        case DIAGNOSTIC_MODE:
            DiagnosticMode();
            break;

        case RESET_MODE:
            /* Turn off all lasers and exposure LED */
            LaserControl(TYPE_HEAD_CEPH, RESET);
            LaserControl(TYPE_HEAD_PANO, RESET);
            LaserControl(TYPE_FOOT, RESET);
            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
            UART_SendMessage(DBG_MSG_PC, "[SP_MODE_RESET]");
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_RESET]");
#endif

            /* Handle tilt-off for ceph model */
            if ((sysInfo.model_id == MODEL_T2_CS) && (g_bTiltStatus == SET)) {
                if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) {
                    printUart(DBG_MSG_PC, "Titing Error");
                }
                g_bTiltStatus = RESET;
            }

            Motor_MoveInitPosition();

#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif
            CurCaptureMode = CAPTURE_CANCEL;
            break;

        case CAPTURE_CANCEL:
        default:
            /* Handle boot error LED indicators */
            if (!g_boot_gen_ok || !g_boot_col_ok || !g_boot_mot_ok) {
                if (!g_boot_mot_ok) {
                    while (!LampTimer_GetStatus()) { }
                    Motor_Lamp_ErrorStatus();
                }
                if (!g_boot_gen_ok) {
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(4, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(5, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(4, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(5, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(0, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(5, 2);
                }
                if (!g_boot_col_ok) {
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(4, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(5, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(0, 10);
                    while (!LampTimer_GetStatus()) { }
                    LampErrorTimer_Enable(5, 2);
                }
            } else if (IntTimer.bLampEnable == SET) {
                Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
                while (!LampTimer_GetStatus()) { }
            }

#ifdef USE_AGING_MODE
#define AGING_INTERVAL_TIME     30000
            if ((g_bAgingMode == SET) && (PrevCaptureMode != CAPTURE_CANCEL)) {
                if (PrevCaptureMode == CAPTURE_PANO) {
                    CurCaptureMode = CAPTURE_CT;
                } else if (PrevCaptureMode == CAPTURE_CT) {
                    CurCaptureMode = CAPTURE_SCAN;
                } else if (PrevCaptureMode == CAPTURE_SCAN) {
                    Column_Control(COLUMN_DOWN);
                    IntTimer_Delay(AGING_INTERVAL_TIME);
                    while (!IntTimer_GetStatus()) { }
                    Column_Control(COLUMN_STOP);
                    Column_Control(COLUMN_UP);
                    IntTimer_Delay(AGING_INTERVAL_TIME);
                    while (!IntTimer_GetStatus()) { }
                    Column_Control(COLUMN_STOP);
                    if (g_bAgingMode == SET) {
                        CurCaptureMode = CAPTURE_PANO;
                    }
                }
            }
#endif /* USE_AGING_MODE */
            break;
        }

        /*
         * CAN message processing (flag set by CAN2_RX0_IRQHandler).
         * Copy message under IRQ disable to prevent ISR overwriting buffer
         * if a second CAN frame arrives during processing.
         */
        if (g_can_rx_flag) {
            __disable_irq();
            CAN_RxMessage = g_can_rx_msg;
            g_can_rx_flag = 0;
            __enable_irq();
            CAN_ParseMessage();
        }

        /*
         * Safety checks and debug log flush every main loop iteration.
         */
        Safety_CheckMainLoop();
        DbgLog_Flush();
    }
}
