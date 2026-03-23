/*
 * arch_tables.c : Panoramic arch trajectory lookup tables
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps the existing arch table data from arch_standard.h, arch_child.h,
 * arch_sinus.h, arch_tmj.h into the clean ArchTable_Get() interface.
 *
 * This file includes the existing headers that define const arrays.
 * The original arch.c copy logic is preserved: Motor_R, Motor_V, and
 * Sensor_P arrays are built from the raw table data at runtime by the
 * capture sequence modules.
 *
 * NOTE: This module references the legacy arch headers which are in inc/.
 * The build system must add inc/ to the include path for Domain/ files
 * that need arch table access.
 *
 * Memory: const tables in Flash (~20KB), ~100 bytes code.
 */

#include "arch_tables.h"

/* Include the raw arch table data headers.
 * These contain const float/int32_t arrays defined at file scope. */
#include "arch_standard.h"
#include "arch_child.h"
#include "arch_sinus.h"
#include "arch_tmj.h"

/* ------------------------------------------------------------------ */

/* Acceleration table size (common to all arch types) */
#define ARCH_ACCEL_SIZE     50U

/* ------------------------------------------------------------------ */
int32_t ArchTable_Get(ArchType_t arch, ArchScan_t scan, ArchTableSet_t* out)
{
    if (out == (void*)0) {
        return -1;
    }

    out->accel_size = ARCH_ACCEL_SIZE;

    switch (arch) {
    case ARCH_ADULT:
        out->capture_size   = StandardCaptureSize;
        out->capture_step   = StandardCaptureStep;
        out->r_start_offset = StandardStartPosRoffset;
        out->v_start_offset = StandardStartPosVoffset;

        if (scan == ARCH_SCAN_HD) {
            out->r_ccr      = StandardRtableHdRate;
            out->r_step     = StandardRtableHdStep;
            out->v_ccr      = StandardVtableHdRate;
            out->v_step     = StandardVtableHdStep;
            out->sensor_ccr = StandardStableHdRate;
        } else {
            out->r_ccr      = StandardRtableNormalRate;
            out->r_step     = StandardRtableNormalStep;
            out->v_ccr      = StandardVtableNormalRate;
            out->v_step     = StandardVtableNormalStep;
            out->sensor_ccr = StandardStableNormalRate;
        }
        break;

    case ARCH_CHILD:
        out->capture_size   = ChildCaptureSize;
        out->capture_step   = ChildCaptureStep;
        out->r_start_offset = ChildStartPosRoffset;
        out->v_start_offset = ChildStartPosVoffset;

        if (scan == ARCH_SCAN_HD) {
            out->r_ccr      = ChildRtableHdRate;
            out->r_step     = ChildRtableHdStep;
            out->v_ccr      = ChildVtableHdRate;
            out->v_step     = ChildVtableHdStep;
            out->sensor_ccr = ChildStableHdRate;
        } else {
            out->r_ccr      = ChildRtableNormalRate;
            out->r_step     = ChildRtableNormalStep;
            out->v_ccr      = ChildVtableNormalRate;
            out->v_step     = ChildVtableNormalStep;
            out->sensor_ccr = ChildStableNormalRate;
        }
        break;

    case ARCH_SINUS:
        out->capture_size   = SinusCaptureSize;
        out->capture_step   = SinusCaptureStep;
        out->r_start_offset = SinusStartPosRoffset;
        out->v_start_offset = SinusStartPosVoffset;

        if (scan == ARCH_SCAN_HD) {
            out->r_ccr      = SinusRtableHdRate;
            out->r_step     = SinusRtableHdStep;
            out->v_ccr      = SinusVtableHdRate;
            out->v_step     = SinusVtableHdStep;
            out->sensor_ccr = SinusStableHdRate;
        } else {
            out->r_ccr      = SinusRtableNormalRate;
            out->r_step     = SinusRtableNormalStep;
            out->v_ccr      = SinusVtableNormalRate;
            out->v_step     = SinusVtableNormalStep;
            out->sensor_ccr = SinusStableNormalRate;
        }
        break;

    case ARCH_TMJ:
        out->capture_size   = TmjCaptureSize;
        out->capture_step   = TmjCaptureStep;
        out->r_start_offset = TmjStartPosRoffset;
        out->v_start_offset = TmjStartPosVoffset;

        if (scan == ARCH_SCAN_HD) {
            out->r_ccr      = TmjRtableHdRate;
            out->r_step     = TmjRtableHdStep;
            out->v_ccr      = TmjVtableHdRate;
            out->v_step     = TmjVtableHdStep;
            out->sensor_ccr = TmjStableHdRate;
        } else {
            out->r_ccr      = TmjRtableNormalRate;
            out->r_step     = TmjRtableNormalStep;
            out->v_ccr      = TmjVtableNormalRate;
            out->v_step     = TmjVtableNormalStep;
            out->sensor_ccr = TmjStableNormalRate;
        }
        break;

    default:
        return -1;
    }

    out->total_size = out->accel_size + out->capture_size;
    return 0;
}
