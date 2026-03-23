/*
*******************************************************************************
* cmd_dispatch.h : Command dispatch and common command definitions
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : Shared command string macros used across cmd_pano, cmd_ct,
*             cmd_scan, cmd_system. Common helper functions.
*
* @ Revision History :
*       1) Extracted from serial.c for modular build --------- 2026-03-24
*******************************************************************************
*/

#ifndef __CMD_DISPATCH_H__
#define __CMD_DISPATCH_H__

/* Includes ---------------------------------------------------------------- */
#include "msg_protocol.h"
#include "uart_hw.h"
#include "system.h"

/* Common command strings shared across multiple handlers ------------------- */

/* Tube parameters */
#define UART_TUBE_VOLTAGE           "[SM_KV___"
#define UART_TUBE_CURRENT           "[SM_MA___"
#define UART_CAPTURE_TIME           "[SM_TIME_"
#define UART_CAPTURE_SIZE           "[SM_SIZE_"

/* X-ray exposure */
#define UART_XRAY_EXPOSURE          "[SM_XRAY_EXPOS]"
#define UART_XRAY_EXPOSURE_STOP     "[SM_XRAY_STOP_]"

/* Motor vertical */
#define UART_VERTICAL_UP            "[SM_VERT_UP___]"
#define UART_VERTICAL_DOWN          "[SM_VERT_DOWN_]"
#define UART_VERTICAL_STOP          "[SM_VERT_STOP_]"

/* Frankfort (laser line) */
#define UART_FRANKFORT_UP           "[SM_FRAN_UP___]"
#define UART_FRANKFORT_DOWN         "[SM_FRAN_DOWN_]"
#define UART_FRANKFORT_STOP         "[SM_FRAN_STOP_]"
#define UART_TABLET_FRANKFORT_UP    "[SM_HORI_UP___]"
#define UART_TABLET_FRANKFORT_DOWN  "[SM_HORI_DOWN_]"
#define UART_TABLET_FRANKFORT_STOP  "[SM_HORI_STOP_]"

/* Temple / Ear rod */
#define UART_TEMPLE_PUSH            "[SM_TEMP_PUSH_]"
#define UART_TEMPLE_RELEASE         "[SM_TEMP_RELEA]"
#define UART_EAR_ROD_PUSH           "[SM_EARR_PUSH_]"
#define UART_EAR_ROD_CHILD_PUSH     "[SM_EARR_PUSHC]"
#define UART_EAR_ROD_RELEASE        "[SM_EARR_RELEA]"
#define UART_EAR_ROD_DONE_FOR_MEMB  "[SP_EARR_DONE_]"

/* Column */
#define UART_COLUMN_UP              "[SM_COLU_UP___]"
#define UART_COLUMN_DOWN            "[SM_COLU_DOWN_]"
#define UART_COLUMN_STOP            "[SM_COLU_STOP_]"

/* Laser beams */
#define UART_LASER_ON               "[SM_LASE_ON___]"
#define UART_LASER_OFF              "[SM_LASE_OFF__]"
#define UART_HEAD_BEAM_ON           "[SM_HEAD_ON___]"
#define UART_HEAD_BEAM_OFF          "[SM_HEAD_OFF__]"
#define UART_FOOT_BEAM_ON           "[SM_FOOT_ON___]"
#define UART_FOOT_BEAM_OFF          "[SM_FOOT_OFF__]"
#define UART_CANINE_BEAM_ON         "[SM_CEPH_LAON_]"
#define UART_CANINE_BEAM_OFF        "[SM_CEPH_LAOFF]"
#define UART_HEAD_BEAM_CUSTOM_ON    "[SM_HEAD_ON__"
#define UART_HEAD_BEAM_CUSTOM_OFF   "[SM_HEAD_OFF_"
#define UART_LED_CUSTOM_ON          "[SM_LAMP_ON__"
#define UART_LED_CUSTOM_OFF         "[SM_LAMP_OFF__]"

/* Beam position */
#define UART_BEAM_POS_UP            "[SM_BM_PO_UP__]"
#define UART_BEAM_POS_DOWN          "[SM_BM_PO_DOWN]"

/* Capture confirm */
#define UART_CAPTURE_READY          "[SM_CAPT_READY]"
#define UART_CAPTURE_CONFIRM        "[SM_CAPT_CONFI]"
#define UART_CAPTURE_CONFIRM_BY_MEMB "[MB_CAPT_CONFI]"
#define UART_MEMBRANE_CAPTURE_CONFIG "[SP_CAPT_CONFI]"

/* Canine / manual offsets */
#define UART_CANINE_UP              "[SM_OFFS_UP___]"
#define UART_CANINE_DOWN            "[SM_OFFS_DOWN_]"
#define UART_CANINE_STOP            "[SM_OFFS_STOP_]"
#define UART_MANUAL_VERTICAL_UP     "[SM_VMUP_"
#define UART_MANUAL_VERTICAL_DOWN   "[SM_VMDW_"
#define UART_MANUAL_AAXIS_UP        "[SM_MAUP_"
#define UART_MANUAL_AAXIS_DOWN      "[SM_MADO_"

/* Tube sound / temp */
#define UART_TUBE_SOUND_OFF         "[SM_TUBE_SDOFF]"
#define UART_GET_TUBE_TEMP          "[SM_GET__TEMP_]"

/* Collimator */
#define UART_COLLIMATOR_OPEN        "[SM_COLI_OPEN_]"
#define UART_COLLIMATOR_CLOSE       "[SM_COLI_CLOSE]"

/* Mode exit */
#define UART_MODE_EXIT              "[SM_MODE_EXIT_]"

/* Arch */
#define UART_ARCH_STANDARD          "[SM_ARCH_STAND]"
#define UART_ARCH_CHILD             "[SM_ARCH_CHILD]"
#define UART_ARCH_CHILD_MEMBRANE    "[SP_ARCH_CHILD]"
#define UART_ARCH_SINUS             "[SM_ARCH_SINUS]"
#define UART_ARCH_TMJ               "[SM_ARCH_TMJ__]"

/* Membrane sound */
#define UART_MEMBRANE_SOUND_ENTRY       "[MB_PANO_MODE_]"
#define UART_MEMBRANE_SOUND_PANO_READY  "[MB_PANO_READY]"
#define UART_MEMBRANE_SOUND_CT_READY    "[MB_CT___READY]"
#define UART_MEMBRANE_SOUND_SCAN_READY  "[MB_SCAN_READY]"
#define UART_MEMBRANE_SOUND_CAPTURING   "[MB_CAPT_START]"
#define UART_MEMBRANE_SOUND_CAPTURE_END "[MB_CAPT_END__]"
#define UART_MEMBRANE_RESET             "[MB_FW___RESET]"
#define UART_MEMBRANE_SEND_INIT         "[MB_SEND_INIT_]"

/* Membrane version */
#define UART_RECEIVE_MEMB_VERSION   "[SP_VER_"

/* Exported functions ------------------------------------------------------ */

/**
 * Check capture parameters for the current mode. Returns SET if valid.
 * Used by UART_CAPTURE_CONFIRM handlers across pano/ct/scan.
 */
bool CMD_CheckCaptureParam(void);

/**
 * Exposure SET handler (shared by calibration mode).
 */
char UART_Exposure_SET(void);

/**
 * Collimator auto-align SET/STOP (used in calibration mode).
 */
char UART_Collimator_Auto_SET(void);
char UART_Collimator_Auto_STOP(void);

/**
 * Membrane version check and init data.
 */
char UART_Memb_Ver_Check(char verFlag);
void UART_Memb_Init_Data(void);

/* Per-mode parameter handler functions (called from capture modules) */
bool UART_SetPanoParam(void);
bool UART_SetCtParam(void);
bool UART_SetCephParam(void);
bool UART_SetGeoAlignParam(void);
bool UART_SensorCalibration(void);

#ifdef USE_I2C_EEPROM
char UART_SetRomParam(void);
#endif /* USE_I2C_EEPROM */
char UART_SetDiagParam(void);

#endif /* __CMD_DISPATCH_H__ */
