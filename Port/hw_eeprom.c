/*
 * hw_eeprom.c : I2C EEPROM driver (delegates to eeprom.c)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~200 bytes Flash, 0 bytes SRAM.
 */

#include "hw_eeprom.h"
#include "eeprom.h"
#include "configuration.h"

void HwEEPROM_Init(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_Init();
#endif
}

uint8_t HwEEPROM_ReadByte(uint8_t addr)
{
#ifdef USE_I2C_EEPROM
    return EEPRom_I2C_Read_Byte(addr);
#else
    (void)addr;
    return 0xFF;
#endif
}

void HwEEPROM_WriteByte(uint8_t addr, uint8_t data)
{
#ifdef USE_I2C_EEPROM
    EEPRom_I2C_Write_Byte(addr, data);
#else
    (void)addr;
    (void)data;
#endif
}

uint16_t HwEEPROM_ReadWord(uint8_t addr)
{
#ifdef USE_I2C_EEPROM
    return EEPRom_I2C_Read_Word(addr);
#else
    (void)addr;
    return 0xFFFF;
#endif
}

void HwEEPROM_WriteWord(uint8_t addr, uint16_t data)
{
#ifdef USE_I2C_EEPROM
    EEPRom_I2C_Write_Word(addr, data);
#else
    (void)addr;
    (void)data;
#endif
}

void HwEEPROM_LoadSysInfo(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_LoadSysInfo();
#endif
}

void HwEEPROM_LoadAlignOffset(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_Load_Align_Offset();
#endif
}

void HwEEPROM_LoadApplyMinusOffset(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_Load_Apply_minus_Offset();
#endif
}

void HwEEPROM_EraseAll(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_EraseAll();
#endif
}

void HwEEPROM_ReadAll(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_ReadAll();
#endif
}

void HwEEPROM_ShowMap(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_ShowMap();
#endif
}
