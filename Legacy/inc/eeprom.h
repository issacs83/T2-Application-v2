/*
*********************************************************************************************
* eeprom.h :
*********************************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. -------------------------------------- 2014-11-16
*       2) V1.1.0  ------------------------------------------- 2015-03-20
*       3) V1.2.0  ------------------------------------------- 2015-09-18
*       4) V1.3.0  ------------------------------------------- 2015-10-28
*********************************************************************************************
*/

/* Define to prevent recursive inclusion ----------------------------------- */
#ifndef __EEPROM_H__
#define __EEPROM_H__


/* Include files ------------------------------------------------------- */


/* Exported typedef --------------------------------------------------- */
/* Exported define ---------------------------------------------------- */
#define kEpStartAddr 0x00

#define EEPROM_VERTICAL_UNIT_ADDR 0x00
#define EEPROM_GANTRY_UNIT_ADDR 0x02
#define EEPROM_COLLIMATOR_UNIT_ADDR 0x04
#define EEPROM_CHINREST_UNIT_ADDR 0x06
#define EEPROM_CEPHALO_UNIT_ADDR 0x08

#define ROM_ALIGN_BASE_ADDR 0x10
#define ROM_SYSTEM_BASE_ADDR 0x40

#define ADDR_SIZE   0x02

typedef enum {
#define ROM_MAP() \
    MAP_FUNC(ROM_RAXIS_ALG_ADDR, ROM_ALIGN_BASE_ADDR + 0x00) \
    MAP_FUNC(ROM_PAXIS_ALG_ADDR, ROM_RAXIS_ALG_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CEPH_2ND_COL_START_ADDR, ROM_PAXIS_ALG_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CANINE_ALIGN_ADDR, ROM_CEPH_2ND_COL_START_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CEPH_RAXIS_ALIGN_ADDR, ROM_CANINE_ALIGN_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CT_RAXIS_ALIGN_ADDR, ROM_CEPH_RAXIS_ALIGN_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CT_PAXIS_ALIGN_ADDR, ROM_CT_RAXIS_ALIGN_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_BOARD_ID_ADDR, ROM_SYSTEM_BASE_ADDR + 0x00) \
    MAP_FUNC(ROM_MODEL_ID_ADDR, ROM_BOARD_ID_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CCAXIS_INIT_DIR_ADDR, ROM_MODEL_ID_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CEPH_2ND_COL_END_ADDR, ROM_CCAXIS_INIT_DIR_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_PANO_CNSAXIS_ALG_ADDR, ROM_CEPH_2ND_COL_END_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_PANO_CWEAXIS_ALG_ADDR, ROM_PANO_CNSAXIS_ALG_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_CT_CNSAXIS_ALG_ADDR, ROM_PANO_CWEAXIS_ALG_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_CT_CWEAXIS_ALG_ADDR, ROM_CT_CNSAXIS_ALG_ADDR + ADDR_SIZE)		\
    MAP_FUNC(ROM_CT_RAXIS_PATIENT_ALG_ADDR, ROM_CT_CWEAXIS_ALG_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_CT_PAXIS_PATIENT_ALG_ADDR, ROM_CT_RAXIS_PATIENT_ALG_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_FIRMWARE_SETTING_ADDR, ROM_CT_PAXIS_PATIENT_ALG_ADDR + ADDR_SIZE)  \
    MAP_FUNC(ROM_CEPH_2ND_ADD_STEP_ADDR, ROM_FIRMWARE_SETTING_ADDR + ADDR_SIZE)    \
    MAP_FUNC(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, ROM_CEPH_2ND_ADD_STEP_ADDR + ADDR_SIZE)   \
    MAP_FUNC(ROM_TEMPLE_SUPPORT_STEP_ADDR, ROM_CEPH_2ND_FAST_ADD_STEP_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_FW_COMPATIBILITY_ADDR, ROM_TEMPLE_SUPPORT_STEP_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_CT_FAST_RAXIS_ALIGN_ADDR, ROM_FW_COMPATIBILITY_ADDR + ADDR_SIZE) \   
    MAP_FUNC(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, ROM_CT_FAST_RAXIS_ALIGN_ADDR + ADDR_SIZE)	\
    MAP_FUNC(ROM_CEPH_FAST_2ND_COL_START_ADDR, ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR + ADDR_SIZE)   \
    MAP_FUNC(ROM_MEMBRANE_SOUND_VALUE_ADDR, ROM_CEPH_FAST_2ND_COL_START_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_MEMBRANE_SELECT_VALUE_ADDR, ROM_MEMBRANE_SOUND_VALUE_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_MEMBRANE_BUTTON_DELAY_ADDR, ROM_MEMBRANE_SELECT_VALUE_ADDR + ADDR_SIZE) \
    MAP_FUNC(ROM_MEMBRANE_CANCEL_DELAY_ADDR, ROM_MEMBRANE_BUTTON_DELAY_ADDR + ADDR_SIZE)
#define MAP_FUNC(num, value) num = value,
    ROM_MAP()
#undef MAP_FUNC
#define MAP_FUNC(num, value) + 1
    ROM_MAP_COUNT = ROM_MAP()
#undef MAP_FUNC
#undef ROM_MAP
} romMapAddr_t;

#define kEpAgingBase 		0x20
#define kEpAgingGenerator 	(kEpAgingBase + 0x00)
#define kEpAgingColumn 		(kEpAgingBase + 0x04)
#define kEpAgingMotorRnTilt	(kEpAgingBase + 0x08)
#define kEpAgingMotorH 		(kEpAgingBase + 0x0C)
#define kEpAgingBeams 		(kEpAgingBase + 0x10)
#define kEpAgingIndicator 	(kEpAgingBase + 0x14)
#define kEpAgingTotalCnt 	(kEpAgingBase + 0x18)

#define kEpVersion 			0xFD  /* eeprom map version */ /* 1byte */
#define kEpCheckSum 		0xFE /* 0x00_01 ~ 0xFC_FD word sum */ /* 2 byte*/

#define kEpEndAddr 			0xFF

#define MEMBRANE_KO_CHILD		1
#define MEMBRANE_KO_MAN			2
#define MEMBRANE_KO_WOMAN		3
#define MEMBRANE_EN_WOMAN		4
#define MEMBRANE_RU_WOMAN		5
#define MEMBRANE_VN_WOMAN		6
#define MEMBRANE_JP_WOMAN		7
#define MEMBRANE_TK_WOMAN		8
#define MEMBRANE_CN_WOMAN		9


/* Exported macro --------------------------------------------------- */
/* Exported variables ------------------------------------------------- */


/* Exported functions ------------------------------------------------- */
uint8_t EEPRom_I2C_Read_Byte(uint8_t addr);
void EEPRom_I2C_Write_Byte(uint8_t addr, uint8_t data);
void EEPRom_LoadSysInfo(void);
void EEPRom_EraseAll(void);
void EEPRom_EraseAddrByte(char addr);
void EEPRom_EraseAddrWord(char addr);
void EEPRom_EraseAddrDword(char addr);
void EEPRom_ReadAll(void);
void EEPRom_SetAddress(char addr);
void EEPRom_SetWordData(unsigned short data);
void EEPRom_StartWriteCmd(void);
void EEPRom_ShowMap(void);
void EEPRom_ModeLoop(void);
void EEPRom_ShowOffset(void);

uint16_t EEPRom_I2C_Read_Word(uint8_t addr);
void EEPRom_I2C_Write_Word(uint8_t addr, uint16_t data);
void EEPRom_I2C_Erase_Word(uint8_t addr, uint16_t data);
void EEPRom_I2C_Direct_Write_Word(uint8_t addr, uint16_t data);
void EEPRom_Init(void);
void EEPRom_Load_Align_Offset(void);
void EEPRom_Load_Apply_minus_Offset(void);



#endif /* __EEPROM_H__ */

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

