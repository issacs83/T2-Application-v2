/*
 * arch_tables.h : Panoramic arch trajectory lookup tables (no HW)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Provides arch table selection and access for panoramic capture modes.
 * The actual LUT data is included from the existing arch_*.h headers
 * which contain const float arrays (kept for backward compatibility).
 *
 * This module wraps the legacy arch.c table-copy logic into a
 * hardware-independent interface.
 *
 * Memory: depends on included arch tables (~20KB Flash for all tables).
 */

#ifndef ARCH_TABLES_H
#define ARCH_TABLES_H

#include <stdint.h>

/* Arch type identifiers (matching PanoArch enum from extern.h) */
typedef enum {
    ARCH_NONE = 0,
    ARCH_ADULT,
    ARCH_CHILD,
    ARCH_SINUS,
    ARCH_TMJ
} ArchType_t;

/* Scan resolution */
typedef enum {
    ARCH_SCAN_NONE = 0,
    ARCH_SCAN_ND,
    ARCH_SCAN_HD
} ArchScan_t;

/* Arch table set: arrays of step and CCR values for R, V motors + sensor */
typedef struct {
    const float*    r_ccr;          /* Motor R CCR array */
    const int32_t*  r_step;         /* Motor R step array */
    const float*    v_ccr;          /* Motor V CCR array */
    const int32_t*  v_step;         /* Motor V step array */
    const float*    sensor_ccr;     /* Sensor P CCR array */
    uint32_t        accel_size;     /* Acceleration entries count */
    uint32_t        capture_size;   /* Capture entries count */
    uint32_t        total_size;     /* Total entries (accel + capture) */
    int32_t         r_start_offset; /* Motor R start position offset */
    int32_t         v_start_offset; /* Motor V start position offset */
    uint32_t        capture_step;   /* Capture step count */
} ArchTableSet_t;

/**
 * Get arch table set for a given arch type and scan resolution.
 * Populates the provided output structure with pointers to the
 * const Flash-resident LUT data and metadata.
 *
 * @param arch  Arch type (ARCH_ADULT, ARCH_CHILD, ARCH_SINUS, ARCH_TMJ)
 * @param scan  Scan resolution (ARCH_SCAN_ND or ARCH_SCAN_HD)
 * @param out   Output table set (pointers into const Flash arrays)
 * @return 0 on success, -1 if arch/scan combination is invalid
 */
int32_t ArchTable_Get(ArchType_t arch, ArchScan_t scan, ArchTableSet_t* out);

#endif /* ARCH_TABLES_H */
