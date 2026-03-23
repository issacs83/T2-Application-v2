/*
 * hw_tmc2660.h : TMC2660 SPI driver (Port layer facade)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Re-exports the tmc2660.h API for Port layer consumers.
 * The actual implementation is in src/tmc2660.c.
 */

#ifndef HW_TMC2660_H
#define HW_TMC2660_H

#include "tmc2660.h"

/* All functions are already declared in tmc2660.h:
 *   TMC2660_SPI_Init()
 *   TMC2660_Init(id)
 *   TMC2660_InitAll()
 *   TMC2660_WriteRegister(id, data)
 *   TMC2660_ReadStatus(id)
 *   TMC2660_SetMicrostep(id, resolution)
 *   TMC2660_SetCurrent(id, run, hold)
 *   TMC2660_SetRunCurrent(id, current)
 *   TMC2660_SetChopperConfig(id, config)
 *   TMC2660_Enable(id)
 *   TMC2660_Disable(id)
 */

#endif /* HW_TMC2660_H */
