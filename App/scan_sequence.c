/*
 * scan_sequence.c : Ceph scan sequence orchestration
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~100 bytes Flash, 0 bytes SRAM.
 */

#include "scan_sequence.h"
#include "extern.h"
#include "system.h"
#include "uart_hw.h"

/* ScanCapture() is defined in src/scan_capture.c */
extern void ScanCapture(void);

void ScanSequence_Run(void)
{
    if (sysInfo.model_id == MODEL_T2_CS) {
        ScanCapture();
    } else {
        printUart(DBG_MSG_PC, "can't not support cephalo");
        CurCaptureMode = CAPTURE_CANCEL;
    }
}
