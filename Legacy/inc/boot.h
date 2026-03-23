/*
 * boot.h
 *
 *  Created on: 2019. 10. 7.
 *      Author: issacs
 */

#ifndef BOOT_H_
#define BOOT_H_

#include <stdio.h>
#include <stdint.h>

/* protection type */
#define ROMBASE							0x1FFF0000	//System memory
#define JUMP	1

#define SCB_FIRST_SECTOR				0x20002FF0
#define COLLIMATOR_FIRST_SECTOR	0x20001190
#define GENERATOR_FIRST_SECTOR	0x0000c08d

/* STM32F207 Sector address */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define USER_BOOT_START_ADDRESS	ADDR_FLASH_SECTOR_0

#define USER_END_ADDRESS         ((ADDR_FLASH_SECTOR_11)-(0x1))//0x080dffff //0x0807ffff//0x0808ffff//ADDR_FLASH_SECTOR_9

typedef enum
{
  BOOT_ERROR    = -1,
	BOOT_OK       = 0,
	BOOT_BUSY     = 1
}Bootloader_StatusTypeDef;

typedef enum {
	BOOTFG_NO_FIRST_BOOTING			= 0x0,
	BOOTFG_FIRST_BOOTING				= 0xFF
}Booting_StatusTypeDef;

typedef enum {
	BOOTFG_APP_TO_BOOTLOADER			= 0x0,
	BOOTFG_BOOTLOADER_TO_APP			= 0x1,
	BOOTFG_WAIT_BOOTLOADER_TO_APP	= 0x2
}Jumping_StatusTypeDef;

typedef enum {
	BOOTFG_WAIT_DOWN_APPLICATION		= 0x0,
	BOOTFG_NO_EXIST_APPLICATION			= 0x1,
	BOOTFG_EXIST_APPLICATION				= 0x2
}Application_StatusTypeDef;

typedef enum
{
	CONN_OK     	= 0,
  CONN_READY    = 1,
	CONN_WAIT	    = 2,
	CONN_DOWN			= 3,
	CONN_ABORT		= 4,
	CONN_NOP	    = 5
}Connect_StatusTypeDef;

typedef struct
{
	uint8_t					OnlyOneBootChk;				//@ Booting_StatusTypeDef
	uint8_t					JumpingStatus;				//@ Jump_StatusTypeDef
	uint8_t					ChkExistingApp;				//@ Application_StatusTypeDef
	uint8_t					ConnectStatus;				//@ Connect_StatusTypeDef
	uint32_t				ChkAppFitstSection[4];		//@ Application first data
	uint32_t				AppFitstSection[4];			//@ When downloding is completed, then load Application first data from flash to this
	// uint32_t				ChkAppFitstSection;		//@ Application first data
	// uint32_t				AppFitstSection;			//@ When downloding is completed, then load Application first data from flash to this
	uint32_t 				appSize;
	uint32_t 				crcCheck;

}	Bootloader_DataTypeDef;

extern Bootloader_DataTypeDef boot;

void LOAD_BOOT_Data( void );
Bootloader_StatusTypeDef CHECK_EXIST_Bootloader( void );
void BOOT_Ram( uint32_t address );
Bootloader_StatusTypeDef CHANGE_BOOT_Mode( Jumping_StatusTypeDef mode );
void BOOT_Prepare( void );
void JUMP_Application( uint32_t address );
uint32_t crc_compute(uint32_t * start_addr, uint32_t size);
#endif /* BOOT_H_ */
