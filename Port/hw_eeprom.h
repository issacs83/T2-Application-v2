/*
 * hw_eeprom.h : I2C EEPROM driver (Port layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps eeprom.c I2C access functions for the Port layer.
 */

#ifndef HW_EEPROM_H
#define HW_EEPROM_H

#include <stdint.h>

/* Initialize EEPROM hardware (I2C bus) */
void HwEEPROM_Init(void);

/* Byte-level access */
uint8_t  HwEEPROM_ReadByte(uint8_t addr);
void     HwEEPROM_WriteByte(uint8_t addr, uint8_t data);

/* Word-level access (16-bit) */
uint16_t HwEEPROM_ReadWord(uint8_t addr);
void     HwEEPROM_WriteWord(uint8_t addr, uint16_t data);

/* Load system configuration from EEPROM */
void HwEEPROM_LoadSysInfo(void);

/* Load alignment offsets */
void HwEEPROM_LoadAlignOffset(void);
void HwEEPROM_LoadApplyMinusOffset(void);

/* Bulk operations */
void HwEEPROM_EraseAll(void);
void HwEEPROM_ReadAll(void);
void HwEEPROM_ShowMap(void);

#endif /* HW_EEPROM_H */
