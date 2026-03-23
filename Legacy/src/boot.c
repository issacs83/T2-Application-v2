/*
 * boot.c
 *
 *  Created on: 2019. 10. 7.
 *      Author: issacs
 */
#include "boot.h"
#include "stm32f2xx_flash.h"
#include "extern.h"
#include "timer.h"
//#include "define.h"
//#include "system.h"

Bootloader_DataTypeDef boot;

/* Variable for bootloader jump */
typedef void (*pFunction)(void);
static pFunction appEntry;
static uint32_t appStack;


void
BOOT_Ram( uint32_t address )
{
	pFunction romboot = (pFunction) *((uint32_t *)(ROMBASE + 4)); // The content

	//__HAL_RCC_SYSCFG_CLK_ENABLE();
	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
		/* Delay after an RCC peripheral clock enabling */
		tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
		(void)(tmpreg);
	} while(0);

	SCB->VTOR = ROMBASE;

	//__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	do {
		SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
		SYSCFG->MEMRMP |= SYSCFG_MEMRMP_MEM_MODE_0;
	}while(0);

	__set_MSP(address);

	romboot();
}

void
LOAD_BOOT_Data( void )
{
	/* Load boot data from flash */
	boot.OnlyOneBootChk 		= (*(uint32_t*)(ADDR_FLASH_SECTOR_4)&0xFF);
	boot.JumpingStatus			= (*(uint32_t*)(ADDR_FLASH_SECTOR_4)>> 8);
	boot.ChkExistingApp 		= (*(uint32_t*)(ADDR_FLASH_SECTOR_4)>> 16);
	boot.ConnectStatus			= (*(uint32_t*)(ADDR_FLASH_SECTOR_4)>> 24);/* Wait until connecting with SCB */

	boot.ChkAppFitstSection[0]	= (*(uint32_t*)(ADDR_FLASH_SECTOR_5 + (sizeof(uint32_t)*0)));
	boot.ChkAppFitstSection[1]	= (*(uint32_t*)(ADDR_FLASH_SECTOR_5 + (sizeof(uint32_t)*1)));
	boot.ChkAppFitstSection[2]	= (*(uint32_t*)(ADDR_FLASH_SECTOR_5 + (sizeof(uint32_t)*2)));
	boot.ChkAppFitstSection[3]	= (*(uint32_t*)(ADDR_FLASH_SECTOR_5 + (sizeof(uint32_t)*3)));

	//다운로드 후, 저장한 SCB 애플리케이션
	boot.AppFitstSection[0] = (*(uint32_t*)(ADDR_FLASH_SECTOR_4 + (sizeof(uint32_t)*1)));
	boot.AppFitstSection[1] = (*(uint32_t*)(ADDR_FLASH_SECTOR_4 + (sizeof(uint32_t)*2)));
	boot.AppFitstSection[2] = (*(uint32_t*)(ADDR_FLASH_SECTOR_4 + (sizeof(uint32_t)*3)));
	boot.AppFitstSection[3] = (*(uint32_t*)(ADDR_FLASH_SECTOR_4 + (sizeof(uint32_t)*4)));

	boot.appSize 						= ( *(uint32_t *)(ADDR_FLASH_SECTOR_4 + (sizeof(uint32_t)*9)));
	boot.crcCheck 					= ( *(uint32_t *)(ADDR_FLASH_SECTOR_4 + (sizeof(uint32_t)*10)));
}

Bootloader_StatusTypeDef
CHANGE_BOOT_Mode( Jumping_StatusTypeDef mode )
{
	FLASH_Status flash_status; 
 
	uint32_t *ramsource;
	uint32_t i=0, destination=0;

	LOAD_BOOT_Data();

	boot.JumpingStatus	= mode; // @BOOTFG_BOOTLOADER_TO_APP, BOOTFG_APP_TO_BOOTLOADER;
	
	destination = ADDR_FLASH_SECTOR_4;

	FLASH_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	flash_status  = FLASH_EraseSector(FLASH_Sector_4, VoltageRange_3);

	if(flash_status == FLASH_COMPLETE)
	{
		ramsource = (uint32_t *) &boot;
		
		/* After eraseing flash, then put some delay for the state satus of flash */
		IntTimer_Delay(10);

		for (i = 0; i < sizeof(Bootloader_DataTypeDef); i++)
		{
			if (FLASH_ProgramWord(destination, *(uint32_t*)(ramsource+i)) == FLASH_COMPLETE)
			{
				/* Validate the written value */
				if (*(uint32_t*)destination != *(uint32_t*)(ramsource+i))
				{
					FLASH_Lock();
					return (BOOT_ERROR);
				}

				/* Increase WORD length */
				destination += 0x04;
			}
			else
			{
				FLASH_Lock();
				return (BOOT_ERROR);
			}
		}
	}
	else
	{
		FLASH_Lock();

		return BOOT_ERROR;
	}

	return BOOT_OK;
}

Bootloader_StatusTypeDef
CHECK_EXIST_Bootloader( void )
{
	uint32_t temp_crc = 0;

	if((boot.appSize != 0xFFFFFFFF) && (boot.appSize != 0x0))
		temp_crc = crc_compute(((uint32_t*)0x08020000), boot.appSize/4);

	/* Check bootloader exist */
	if( (boot.ChkAppFitstSection[0] == 0xFFFFFFFF) && (boot.ChkAppFitstSection[1] == 0xFFFFFFFF)
			&& (boot.ChkAppFitstSection[2] == 0xFFFFFFFF) && (boot.ChkAppFitstSection[3] == 0xFFFFFFFF) )
	{
		boot.ChkExistingApp			=	BOOTFG_NO_EXIST_APPLICATION;
		printf("\rNo exist application, Waiting......\r\n\n");
	}
	else if (temp_crc == boot.crcCheck)
	{
		boot.ChkExistingApp			=	BOOTFG_EXIST_APPLICATION;
		printf("\rExist application\r\n\n");
	}
	else
	{
		boot.ChkExistingApp	=	BOOTFG_NO_EXIST_APPLICATION;
		boot.JumpingStatus 	= BOOTFG_APP_TO_BOOTLOADER;
		printf("\rExist application but wrong application so, Waiting......\r\n\n");

		return BOOT_ERROR;
	}

	return BOOT_OK;
}
/**
 * @brief
 * @param  none
 * @param  none
 * @retval BOOT_OK : normally return
 */
void
BOOT_Prepare( void )
{
	printf("\rCollimator Bootloader START!\n");

	LOAD_BOOT_Data();

	if(CHECK_EXIST_Bootloader() == BOOT_OK)
	{
		/* Routine - First booting, Do Only 1 time */
		if(boot.OnlyOneBootChk == BOOTFG_FIRST_BOOTING)
		{
			printf("\rFirst BOOT routine!\n");
			boot.OnlyOneBootChk			= BOOTFG_NO_FIRST_BOOTING;	/* Clear OnlyOnebootChk flag - Next time, It will not enter this routine */
			boot.JumpingStatus			= BOOTFG_APP_TO_BOOTLOADER;	/* Set jump app flag */
		}
	}
}

/**
 * @brief  Jump from bootloader to application
 * @param  None
 * @retval None
 */
void
JUMP_Application( uint32_t address )
{
	CHANGE_BOOT_Mode(BOOTFG_BOOTLOADER_TO_APP);

	printf("\rStart program execution...\r\n");

	/* Get the application stack pointer (First entry in the application vector table) */
	appStack = (uint32_t) *((__IO uint32_t*)address);

	/* Get the application entry point (Second entry in the application vector table) */
	appEntry = (pFunction) *(__IO uint32_t*)(address + 4);

	/* Reconfigure vector table offset register to match the application location */
	SCB->VTOR = address;

	/* Set the application stack pointer */
	__set_MSP(appStack);

	/* Start the application */
	appEntry();
}

/**
 * @brief  Check the bootloader cheksum
 * @param  None
 * @retval None
 */
uint32_t crc_compute(uint32_t * start_addr, uint32_t size)
{
	uint32_t crc_code;
	uint32_t i = 0;

	// Enable CRC
//	__HAL_RCC_CRC_CLK_ENABLE();
	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN);
		/* Delay after an RCC peripheral clock enabling */
		tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN);
//		UNUSED(tmpreg);
		(void)(tmpreg);
	} while(0);

	CRC->CR |= CRC_CR_RESET;

	while(i < size)
	{
		CRC->DR = start_addr[i];
		i++;
	}
	crc_code = CRC->DR;

	// Disable CRC
//	__HAL_RCC_CRC_CLK_DISABLE();
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_CRCEN);

	return crc_code;

}
