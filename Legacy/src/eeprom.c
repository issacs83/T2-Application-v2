/*
*******************************************************************************
* eeprom.c :
*******************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------- 2014-11-16
*       2) V1.1.0  ------------------------------------------- 2015-03-20
*       3) V1.2.0  ------------------------------------------- 2015-09-18
*       4) V1.3.0  ------------------------------------------- 2015-10-28
*******************************************************************************
*/

/* Include files ------------------------------------------------------ */
#include "extern.h"
#include "system.h"
#include "motor.h"
#include "serial.h"
#include "timer.h"

#ifdef USE_I2C_EEPROM
#include "eeprom.h"

/* Private typedef --------------------------------------------------- */
typedef struct {
    char addr;
    unsigned short data;
} romParam_t;

typedef struct {
    const char *name;
    const char addr;
} romMap_t;


/* Private define ---------------------------------------------------- */
#define WAIT_FOR_EVENT_TIME 		2000000
#define I2C_OwnAddr  				0xA0 

#define CT_ALIGN_POS_PAXIS_DIAMETER      	45.0
#define CT_START_POS_RAXIS_DEGREE	        0.0

#define	CT_ALIGN_POS_PAXIS_OFFSET	 	    (CT_ALIGN_POS_PAXIS_DIAMETER / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define CT_START_POS_RAXIS_OFFSET        	(CT_START_POS_RAXIS_DEGREE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static romParam_t romParam;

static const romMap_t map[] = {
    { "Pano R-Axis Align Offset", ROM_RAXIS_ALG_ADDR },    
    { "Pano P-Axis Align Offset", ROM_PAXIS_ALG_ADDR },
    { "Ceph 2ND Col Start Offset", ROM_CEPH_2ND_COL_START_ADDR },
    { "Canine Align Offset", ROM_CANINE_ALIGN_ADDR },
    { "Ceph R-Axis Align Offset", ROM_CEPH_RAXIS_ALIGN_ADDR },
    { "CT R-Axis Align Offset", ROM_CT_RAXIS_ALIGN_ADDR },
    { "CT P-Axis Align Offset", ROM_CT_PAXIS_ALIGN_ADDR },
    { "Board ID", ROM_BOARD_ID_ADDR },
    { "Model ID", ROM_MODEL_ID_ADDR },
    { "Init Dir for CC-Axis", ROM_CCAXIS_INIT_DIR_ADDR },
    { "Ceph 2ND Col End Offset",ROM_CEPH_2ND_COL_END_ADDR},
    { "Pano CNS-Axis Align Offset",ROM_PANO_CNSAXIS_ALG_ADDR},
    { "Pano CWE-Axis Align Offset",ROM_PANO_CWEAXIS_ALG_ADDR},
    { "CT CNS-Axis Align Offset",ROM_CT_CNSAXIS_ALG_ADDR},
    { "CT CWE-Axis Align Offset",ROM_CT_CWEAXIS_ALG_ADDR},
    { "CT R-Axis Patient Align Offset",ROM_CT_RAXIS_PATIENT_ALG_ADDR},
    { "CT P-Axis Patient Align Offset",ROM_CT_PAXIS_PATIENT_ALG_ADDR},
    { "FIRMWARE SETTING VALUE",ROM_FIRMWARE_SETTING_ADDR},
    { "Ceph 2ND Col Add Step Offset",ROM_CEPH_2ND_ADD_STEP_ADDR},
    { "Ceph 2ND Col Fast Add Step Offset",ROM_CEPH_2ND_FAST_ADD_STEP_ADDR},
    { "Temple Support Step Offset",ROM_TEMPLE_SUPPORT_STEP_ADDR},
    { "FW Compatibility value",ROM_FW_COMPATIBILITY_ADDR},
    { "CT FAST R-Axis Align Offset", ROM_CT_FAST_RAXIS_ALIGN_ADDR },
    { "Temple Support Child Step Offset",ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR},
    { "Ceph Fast 2ND Col Start Offset",ROM_CEPH_FAST_2ND_COL_START_ADDR},
    { "Membrane Sound Value",ROM_MEMBRANE_SOUND_VALUE_ADDR},
    { "Membrane Sound Select",ROM_MEMBRANE_SELECT_VALUE_ADDR},
    { "Membrane Button Delay Count",ROM_MEMBRANE_BUTTON_DELAY_ADDR},
    { "Membrane Cancel Delay Count",ROM_MEMBRANE_CANCEL_DELAY_ADDR},
};


/* Private function prototypes ------------------------------------------*/
static void I2C_Initialize(void);
static uint32_t I2C_WaitForEvent(uint32_t event);
static void I2C_Send_Start(void);
static void I2C_Send_Stop(void);
static void I2C_Send_Data(uint8_t data);
static void I2C_Send_Data_R(uint8_t data);
static uint8_t I2C_Receive_Byte(void);
static uint32_t EEPRom_I2C_Read_Dword(uint8_t addr);
static void EEPRom_I2C_Write_Dword(uint8_t addr, uint32_t data);


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : I2C_Initialize
* @ Desc : 
* @ Param : 
* @ Return :
*/
void I2C_Initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /* Enable I2C2 and I2C2_PORT & Alternate Function clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Reset I2C2 IP */
    //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);

    /* Release reset signal of I2C2 IP */
    //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);  

    /* I2C2 SCL and SDA pins configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_UP; //GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    
    /* Alternate function remapping */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2); // SDA

    /* I2C De-initialize */
    I2C_DeInit(I2C2);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;

    I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;//100000;
    I2C_Init(I2C2, &I2C_InitStructure);

    /* I2C ENABLE */
    I2C_Cmd(I2C2, ENABLE);
}

/**
* @ Function Name : I2C_WaitForEvent
* @ Desc : 
* @ Param : 
* @ Return :
*/
static uint32_t I2C_WaitForEvent(uint32_t event) 
{      
    int cnt;
    
    for (cnt = WAIT_FOR_EVENT_TIME; cnt > 0; cnt--) 
    {      
        uint32_t sr_reg = I2C2->SR1;      
        if (sr_reg & event) {
            return sr_reg;
        }
    }
    
    return 0;      
}      

/**
* @ Function Name : I2C_Send_Start
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void I2C_Send_Start(void) 
{      
    I2C_GenerateSTART(I2C2, ENABLE); // send start when bus becomes available       

    I2C_WaitForEvent(I2C_FLAG_SB);
    I2C2->DR = I2C_OwnAddr; // send slave address for transmission       

    I2C_WaitForEvent(I2C_FLAG_ADDR);      
    I2C2->SR2; // clear event       
}

/**
* @ Function Name : I2C_Send_Stop
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void I2C_Send_Stop(void) 
{      
    I2C_WaitForEvent(I2C_FLAG_TXE);      
    I2C_GenerateSTOP(I2C2, ENABLE); // send stop after current byte       
}

/**
* @ Function Name : I2C_Send_Data
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void I2C_Send_Data(uint8_t data)
{      
    //I2C_WaitForEvent(I2C_FLAG_TXE);       
    I2C_WaitForEvent(I2C_FLAG_BTF);      
    I2C2->DR = data; // send byte       
}

static void I2C_Send_Data_R(uint8_t data)
{      
    //I2C_WaitForEvent(I2C_FLAG_TXE);       
    //I2C_WaitForEvent(I2C_FLAG_BTF);      
    I2C2->DR = data; // send byte       
}


/**
* @ Function Name : I2C_Receive_Byte
* @ Desc : 
* @ Param : 
* @ Return :
*/
static uint8_t I2C_Receive_Byte(void) 
{      
    I2C_GenerateSTART(I2C2, ENABLE);  // send start when bus becomes available     
    I2C_WaitForEvent(I2C_FLAG_SB);      
    I2C_AcknowledgeConfig(I2C2, DISABLE); // only one byte will be read       
    I2C2->DR = I2C_OwnAddr | 0x01; // send slave address for reception     
    I2C_WaitForEvent(I2C_FLAG_ADDR);
    I2C2->SR2; // clear event       

    I2C_WaitForEvent(I2C_FLAG_RXNE);      
    I2C_GenerateSTOP(I2C2, ENABLE);  // send stop after current byte
    return I2C2->DR; // receive byte       
}      

/**
* @ Function Name : EEPRom_I2C_Read_Byte
* @ Desc : 
* @ Param : 
* @ Return :
*/
uint8_t EEPRom_I2C_Read_Byte(uint8_t addr) 
{      
    /* Enable I2C2 acknowledgement if it is already disabled by other function */      
    //I2C_AcknowledgeConfig(I2C2, ENABLE);       
    I2C_Send_Start();   
    I2C_Send_Data_R(addr & 0xFF);      
    //I2C_Send_Start();       
    //I2C_Send_Stop();
    
    return I2C_Receive_Byte();      
}      

/**
* @ Function Name : EEPRom_I2C_Read_Word
* @ Desc : 
* @ Param : 
* @ Return :
*/
uint16_t EEPRom_I2C_Read_Word(uint8_t addr)
{
	uint16_t data=0xffff;
	sysInfo.bIf0xffff=RESET;

	data=(uint16_t)((EEPRom_I2C_Read_Byte(addr) << 8 ) & 0xff00) | (uint16_t)(EEPRom_I2C_Read_Byte(addr + 1) & 0x00ff);

	if((data!=0xffff) && ((data & 0x8000) == 0x8000))
	{
		data+=1;
		if(data==0xffff)
		{
			sysInfo.bIf0xffff=SET;
		}
	}
	return data;
}

/**
* @ Function Name : EEPRom_I2C_Write_Byte
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_I2C_Write_Byte(uint8_t addr, uint8_t data) 
//static void EEPRom_I2C_Write_Byte(uint8_t addr, uint8_t data) 
{      
    /* Enable I2C2 acknowledgement if it is already disabled by other function */      
    //I2C_AcknowledgeConfig(I2C2, ENABLE);       
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
        
    I2C_Send_Start();
    
    I2C_Send_Data(addr & 0xFF);
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read
    I2C_Send_Data(data);
    
    I2C_WaitForEvent(I2C_FLAG_BTF);
    I2C_Send_Stop();
} 

/**
* @ Function Name : EEPRom_I2C_Write_Word
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_I2C_Write_Word(uint8_t addr, uint16_t data)
{  
    /* Enable I2C2 acknowledgement if it is already disabled by other function */      
    //I2C_AcknowledgeConfig(I2C2, ENABLE);       
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
    if((data & 0x8000) == 0x8000)
	{
		if(data!=0x8000 && data!=0x8001) 
			data-=2;
	}
        
    I2C_Send_Start();      
    
    I2C_Send_Data(addr & 0xFF);
    
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
    I2C_Send_Data((uint8_t)((data >> 8) & 0x00ff));   
    I2C_Send_Data((uint8_t)((data) & 0x00ff));   
    
    I2C_WaitForEvent(I2C_FLAG_BTF);      
    
    I2C_Send_Stop();
}
/**
* @ Function Name : EEPRom_I2C_Write_Word
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_I2C_Erase_Word(uint8_t addr, uint16_t data)
{  
    /* Enable I2C2 acknowledgement if it is already disabled by other function */      
    //I2C_AcknowledgeConfig(I2C2, ENABLE);       
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
        
    I2C_Send_Start();      
    
    I2C_Send_Data(addr & 0xFF);
    
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
    I2C_Send_Data((uint8_t)((data >> 8) & 0x00ff));   
    I2C_Send_Data((uint8_t)((data) & 0x00ff));   
    
    I2C_WaitForEvent(I2C_FLAG_BTF);      
    
    I2C_Send_Stop();
}

void EEPRom_I2C_Direct_Write_Word(uint8_t addr, uint16_t data)
{  
    /* Enable I2C2 acknowledgement if it is already disabled by other function */      
    //I2C_AcknowledgeConfig(I2C2, ENABLE);       
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
	if((data!=0xffff) && ((data & 0x8000) == 0x8000))
	{
		if(data!=0x8000 && data!=0x8001) 
			data-=1;
	}
        
    I2C_Send_Start();      
    
    I2C_Send_Data(addr & 0xFF);
    
    //I2C_AcknowledgeConfig(I2C2, DISABLE);  // only one byte will be read       
    I2C_Send_Data((uint8_t)((data >> 8) & 0x00ff));   
    I2C_Send_Data((uint8_t)((data) & 0x00ff));   
    
    I2C_WaitForEvent(I2C_FLAG_BTF);      
    
    I2C_Send_Stop();
}

/**
* @ Function Name : EEPRom_I2C_Dwead_Word
* @ Desc : 
* @ Param : 
* @ Return :
*/
static uint32_t EEPRom_I2C_Read_Dword(uint8_t addr)
{
	return (uint32_t) ( (uint32_t)((EEPRom_I2C_Read_Byte(addr + 0) << (8*0) ) & 0x000000ff) 
		              | (uint32_t)((EEPRom_I2C_Read_Byte(addr + 1) << (8*1) ) & 0x0000ff00)
 		              | (uint32_t)((EEPRom_I2C_Read_Byte(addr + 2) << (8*2) ) & 0x00ff0000)
		              | (uint32_t)((EEPRom_I2C_Read_Byte(addr + 3) << (8*3) ) & 0xff000000) );
}

/**
* @ Function Name : EEPRom_I2C_Write_Dword
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void EEPRom_I2C_Write_Dword(uint8_t addr, uint32_t data)
{  
    I2C_Send_Start();      
    
    I2C_Send_Data(addr & 0xFF);
    
    I2C_Send_Data((uint8_t)((data >> (8*0) ) & 0x000000ff));
    I2C_Send_Data((uint8_t)((data >> (8*1) ) & 0x000000ff));
    I2C_Send_Data((uint8_t)((data >> (8*2) ) & 0x000000ff));
    I2C_Send_Data((uint8_t)((data >> (8*3) ) & 0x000000ff));	
    
    I2C_WaitForEvent(I2C_FLAG_BTF);      
    
    I2C_Send_Stop();
}


/**
* @ Function Name : EEPRom_Init
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_Init(void)
{
    /* Initialize the I2C driver */
    I2C_Initialize();
}

/**
* @ Function Name : EEPRom_Load_Align_Offset
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_Load_Align_Offset(void)
{	
	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "\r\n");
		printUart(DBG_MSG_PC, "ROM_RAXIS_ALG_ADDR(0x%02x) : [0x%04x]", 
	            ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
	}
    PanoParam.nRAxisOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
    if (PanoParam.nRAxisOffset == (int16_t)0xffff)
    {
        PanoParam.nRAxisOffset = 0;
    }    
	else if(PanoParam.nRAxisOffset<0) PanoParam.nRAxisOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "ROM_PAXIS_ALG_ADDR(0x%02x) : [0x%04x]", 
		        ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
	}
    PanoParam.nVAxisOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
    if (PanoParam.nVAxisOffset == (int16_t)0xffff)
    {
        PanoParam.nVAxisOffset = 0;
    } 
	else if(PanoParam.nVAxisOffset<0) PanoParam.nVAxisOffset++;

    if (sysInfo.model_id == MODEL_T2_CS)
    {
		if(sysInfo.bShowLog==TRUE)
		{
	    	printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
		}
        CephParam.n2ndColOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR);
        if (CephParam.n2ndColOffset == (int16_t)0xffff)
        {
            CephParam.n2ndColOffset = 0;
        }
		else if(CephParam.n2ndColOffset <0) CephParam.n2ndColOffset++;

		if(sysInfo.bShowLog==TRUE)
		{
	    	printUart(DBG_MSG_PC, "ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_FAST_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR));
		}
	    CephParam.n2ndColFastStartOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR);
	    if (CephParam.n2ndColFastStartOffset == (int16_t)0xffff)
	    {
	        CephParam.n2ndColFastStartOffset = CephParam.n2ndColOffset;
	    }
		else if(CephParam.n2ndColFastStartOffset<0) CephParam.n2ndColFastStartOffset++;
		

		if(sysInfo.bShowLog==TRUE)
		{
		    printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
		            ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
		}
	    CephParam.nRAxisOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR);
	    if (CephParam.nRAxisOffset == (int16_t)0xffff)
	    {
	        CephParam.nRAxisOffset = 0;
	    }
		else if(CephParam.nRAxisOffset<0) CephParam.nRAxisOffset++;
    }
    
	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "ROM_CANINE_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CANINE_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR));
	}
    PanoParam.nCanineOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR);    
    if (PanoParam.nCanineOffset == (int16_t)0xffff)
    {
        PanoParam.nCanineOffset = MOTOR_V_TO_LIMIT_RUN_STEP;
    }    
	else if(PanoParam.nCanineOffset<0) PanoParam.nCanineOffset++;
   
	

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "CT_RAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR));
	}
    CtParam.nRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR);
    if (CtParam.nRaxisOffset == (signed short)0xffff)
    {
        CtParam.nRaxisOffset = 0;
    }
	else if(CtParam.nRaxisOffset<0) CtParam.nRaxisOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "ROM_CT_FAST_RAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_FAST_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR));
	}
    CtParam.nFastRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR);
    if (CtParam.nFastRaxisOffset == (signed short)0xffff)
    {
        CtParam.nFastRaxisOffset = 0;
    }
	else if(CtParam.nFastRaxisOffset<0) CtParam.nFastRaxisOffset++;
	

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "CT_PAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_PAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR));
	}
    CtParam.nPaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR);
    if (CtParam.nPaxisOffset == (signed short)0xffff)
    {
        CtParam.nPaxisOffset = 0;
    }
	else if(CtParam.nPaxisOffset<0) CtParam.nPaxisOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "PANO_CNSAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
         	   ROM_PANO_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR));
	}
    PanoParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR);
    if (PanoParam.nCNSaxisOffset== (signed short)0xffff)
    {
        PanoParam.nCNSaxisOffset = 0;
    }
	else if(PanoParam.nCNSaxisOffset<0) PanoParam.nCNSaxisOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "PANO_CWEAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_PANO_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR));
	}
    PanoParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR);
    if (PanoParam.nCWEaxisOffset == (signed short)0xffff)
    {
        PanoParam.nCWEaxisOffset = 0;
    }
	else if(PanoParam.nCWEaxisOffset<0) PanoParam.nCWEaxisOffset++;
	

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "CT_CNSAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR));
	}
    CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
    if (CtParam.nCNSaxisOffset== (signed short)0xffff)
    {
        CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset;
    }
	else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "CT_CWEAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR));
	}
    CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
    if (CtParam.nCWEaxisOffset == (signed short)0xffff)
    {
        CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset;
    }
	else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
	
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "ROM_CT_RAXIS_PATIENT_ALG_ADDR(0x%02x) : [0x%04x]", 
            ROM_CT_RAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR));
	}
	CtParam.nRaxisPatientOffset = EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
	if (CtParam.nRaxisPatientOffset == (signed short)0xffff)
    {
        CtParam.nRaxisPatientOffset = 0;
    }
	else if(CtParam.nRaxisPatientOffset <0) CtParam.nRaxisPatientOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "ROM_CT_PAXIS_PATIENT_ALG_ADDR(0x%02x) : [0x%04x]", 
            ROM_CT_PAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR));
	}
	CtParam.nPaxisPatientOffset = EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
	if (CtParam.nPaxisPatientOffset == (signed short)0xffff)
    {
        CtParam.nPaxisPatientOffset = 0;
    }
	else if(CtParam.nPaxisPatientOffset<0) CtParam.nPaxisPatientOffset++;

	if (sysInfo.model_id == MODEL_T2_CS)
    {
    	//191224 HWAN Ceph
		if(sysInfo.bShowLog==TRUE)
		{
	    	printUart(DBG_MSG_PC, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));
		}
        CephParam.n2ndSpeedStepOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR);
        if (CephParam.n2ndSpeedStepOffset == (int16_t)0xffff)
        {
            CephParam.n2ndSpeedStepOffset = 0;
        }
		else if(CephParam.n2ndSpeedStepOffset<0) CephParam.n2ndSpeedStepOffset++;
        
		if(sysInfo.bShowLog==TRUE)
		{
	    	printUart(DBG_MSG_PC, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
		}
        CephParam.n2ndFastSpeedStepOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR);
        if (CephParam.n2ndFastSpeedStepOffset == (int16_t)0xffff)
        {
            CephParam.n2ndFastSpeedStepOffset = CephParam.n2ndSpeedStepOffset;
        }
		else if(CephParam.n2ndFastSpeedStepOffset<0) CephParam.n2ndFastSpeedStepOffset++;
		
		//190730 HWAN Ceph offset
		if(sysInfo.bShowLog==TRUE)
		{
		    printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : [0x%04x]", 
		            ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));
		}	
		CephParam.n2ndColEndOffset = EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR);
		if (CephParam.n2ndColEndOffset == (signed short)0xffff)
		{
			CephParam.n2ndColEndOffset = 0;
		}
		else if(CephParam.n2ndColEndOffset<0) CephParam.n2ndColEndOffset++;
    }

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_STEP_ADDR(0x%02x) : [0x%04x]", 
    	        ROM_TEMPLE_SUPPORT_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR));
	}
    PanoParam.nTAxisOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR);
    if (PanoParam.nTAxisOffset == (int16_t)0xffff)
    {
        PanoParam.nTAxisOffset = 0;
    }
	else if(PanoParam.nTAxisOffset<0) PanoParam.nTAxisOffset++;

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR(0x%02x) : [0x%04x]", 
    	        ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR));
	}
    PanoParam.nTAxisChildOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR);
    if (PanoParam.nTAxisChildOffset == (int16_t)0xffff)
    {
        PanoParam.nTAxisChildOffset = 1000;
    }
	else if(PanoParam.nTAxisChildOffset<0) PanoParam.nTAxisChildOffset++;
	
	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ROM_MEMBRANE_SOUND_VALUE_ADDR(0x%02x) : [0x%04x]", 
    	        ROM_MEMBRANE_SOUND_VALUE_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_SOUND_VALUE_ADDR));
	}
    sysInfo.nMembrane_Sound_Value= (int16_t)EEPRom_I2C_Read_Word(ROM_MEMBRANE_SOUND_VALUE_ADDR);
    if (sysInfo.nMembrane_Sound_Value == (int16_t)0xffff || sysInfo.nMembrane_Sound_Value>254)
    {
        sysInfo.nMembrane_Sound_Value = 240;
    }

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ROM_MEMBRANE_SELECT_VALUE_ADDR(0x%02x) : [0x%04x]", 
    	        ROM_MEMBRANE_SELECT_VALUE_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_SELECT_VALUE_ADDR));
	}
    sysInfo.nMembrane_Sound_Select = (int16_t)EEPRom_I2C_Read_Word(ROM_MEMBRANE_SELECT_VALUE_ADDR);
    if (sysInfo.nMembrane_Sound_Select  == (int16_t)0xffff || sysInfo.nMembrane_Sound_Select>100)
    {
        sysInfo.nMembrane_Sound_Select  = MEMBRANE_KO_WOMAN;
    }

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ROM_MEMBRANE_BUTTON_DELAY_ADDR(0x%02x) : [0x%04x]", 
    	        ROM_MEMBRANE_BUTTON_DELAY_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_BUTTON_DELAY_ADDR));
	}
    sysInfo.nMembrane_Button_Delay= (int16_t)EEPRom_I2C_Read_Word(ROM_MEMBRANE_BUTTON_DELAY_ADDR);
    if (sysInfo.nMembrane_Button_Delay == (int16_t)0xffff || sysInfo.nMembrane_Button_Delay>10000)
    {
        sysInfo.nMembrane_Button_Delay = 100;
    }

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ROM_MEMBRANE_CANCEL_DELAY_ADDR(0x%02x) : [0x%04x]", 
    	        ROM_MEMBRANE_CANCEL_DELAY_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_CANCEL_DELAY_ADDR));
	}
    sysInfo.nMembrane_Cancel_Delay= (int16_t)EEPRom_I2C_Read_Word(ROM_MEMBRANE_CANCEL_DELAY_ADDR);
    if (sysInfo.nMembrane_Cancel_Delay == (int16_t)0xffff || sysInfo.nMembrane_Cancel_Delay>10000)
    {
        sysInfo.nMembrane_Cancel_Delay = 3000;
    }
	
}

void EEPRom_Load_Apply_minus_Offset(void)
{	
	printUart(DBG_MSG_PC, "\r\n");
	printUart(DBG_MSG_PC, "ROM_RAXIS_ALG_ADDR(0x%02x) : [0x%04x]", 
	            ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
	
    PanoParam.nRAxisOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
    if (PanoParam.nRAxisOffset == (int16_t)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	PanoParam.nRAxisOffset = 0;
		else
		{
			PanoParam.nRAxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)PanoParam.nRAxisOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
		}
    }    
	else if((PanoParam.nRAxisOffset & 0x8000)==0x8000)
	{
		if(PanoParam.nRAxisOffset != 0x8000)
		{
			PanoParam.nRAxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)PanoParam.nRAxisOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "ROM_PAXIS_ALG_ADDR(0x%02x) : [0x%04x]", 
		        ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
	
    PanoParam.nVAxisOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
    if (PanoParam.nVAxisOffset == (int16_t)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	PanoParam.nVAxisOffset = 0;
		else
		{
			PanoParam.nVAxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)PanoParam.nVAxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    } 	
	else if((PanoParam.nVAxisOffset & 0x8000)==0x8000)
	{
		if(PanoParam.nVAxisOffset != 0x8000)
		{
			PanoParam.nVAxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)PanoParam.nVAxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

    if (sysInfo.model_id == MODEL_T2_CS)
    {
		printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
		
        CephParam.n2ndColOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR);
        if (CephParam.n2ndColOffset == (int16_t)0xffff)
        {
        	if(sysInfo.bIf0xffff==RESET)
            	CephParam.n2ndColOffset = 0;
			else
			{
				CephParam.n2ndColOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_START_ADDR, (uint16_t)CephParam.n2ndColOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
        }		
		else if((CephParam.n2ndColOffset & 0x8000)==0x8000)
		{
			if(CephParam.n2ndColOffset != 0x8000)
			{
				CephParam.n2ndColOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_START_ADDR, (uint16_t)CephParam.n2ndColOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
		}

		printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
		            ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
	    CephParam.nRAxisOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR);
	    if (CephParam.nRAxisOffset == (int16_t)0xffff)
	    {
	    	if(sysInfo.bIf0xffff==RESET)
	        	CephParam.nRAxisOffset = 0;
			else
			{
				CephParam.nRAxisOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)CephParam.nRAxisOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
	    }
		else if((CephParam.nRAxisOffset & 0x8000)==0x8000)
		{
			if(CephParam.nRAxisOffset != 0x8000)
			{
				CephParam.nRAxisOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)CephParam.nRAxisOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
		}
    }
    
	printUart(DBG_MSG_PC, "ROM_CANINE_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CANINE_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR));
	
    PanoParam.nCanineOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR);    
    if (PanoParam.nCanineOffset == (int16_t)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	PanoParam.nCanineOffset = MOTOR_V_TO_LIMIT_RUN_STEP;
		else
		{
			PanoParam.nCanineOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CANINE_ALIGN_ADDR, (uint16_t)PanoParam.nCanineOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    } 
	else if((PanoParam.nCanineOffset & 0x8000)==0x8000)
	{
		if(PanoParam.nCanineOffset != 0x8000)
		{
			PanoParam.nCanineOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CANINE_ALIGN_ADDR, (uint16_t)PanoParam.nCanineOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "ROM_CT_RAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR));
    CtParam.nRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR);
    if (CtParam.nRaxisOffset == (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	CtParam.nRaxisOffset = CT_START_POS_RAXIS_OFFSET;
		else
		{
			CtParam.nRaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_RAXIS_ALIGN_ADDR, (uint16_t)CtParam.nRaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }
	else if((CtParam.nRaxisOffset & 0x8000)==0x8000)
	{
		if(CtParam.nRaxisOffset != 0x8000)
		{
			CtParam.nRaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_RAXIS_ALIGN_ADDR, (uint16_t)CtParam.nRaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "ROM_CT_PAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_PAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR));
    CtParam.nPaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR);
    if (CtParam.nPaxisOffset == (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
       		CtParam.nPaxisOffset = 0;
		else
		{
			CtParam.nPaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_PAXIS_ALIGN_ADDR, (uint16_t)CtParam.nPaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }	
	else if((CtParam.nPaxisOffset & 0x8000)==0x8000)
	{
		if(CtParam.nPaxisOffset != 0x8000)
		{
			CtParam.nPaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_PAXIS_ALIGN_ADDR, (uint16_t)CtParam.nPaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "PANO_CNSAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
         	   ROM_PANO_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR));
    PanoParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR);
    if (PanoParam.nCNSaxisOffset== (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	PanoParam.nCNSaxisOffset = 0;
		else
		{
			PanoParam.nCNSaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_PANO_CNSAXIS_ALG_ADDR, (uint16_t)PanoParam.nCNSaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }	
	else if((PanoParam.nCNSaxisOffset & 0x8000)==0x8000)
	{
		if(PanoParam.nCNSaxisOffset != 0x8000)
		{
			PanoParam.nCNSaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_PANO_CNSAXIS_ALG_ADDR, (uint16_t)PanoParam.nCNSaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "PANO_CWEAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_PANO_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR));
    PanoParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR);
    if (PanoParam.nCWEaxisOffset == (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	PanoParam.nCWEaxisOffset = 0;
		else
		{
			PanoParam.nCWEaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_PANO_CWEAXIS_ALG_ADDR, (uint16_t)PanoParam.nCWEaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }
	else if((PanoParam.nCWEaxisOffset & 0x8000)==0x8000)
	{
		if(PanoParam.nCWEaxisOffset != 0x8000)
		{
			PanoParam.nCWEaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_PANO_CWEAXIS_ALG_ADDR, (uint16_t)PanoParam.nCWEaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}
	

	printUart(DBG_MSG_PC, "CT_CNSAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR));
    CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
    if (CtParam.nCNSaxisOffset== (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset;
		else
		{
			CtParam.nCNSaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_CNSAXIS_ALG_ADDR, (uint16_t)CtParam.nCNSaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }
	else if((CtParam.nCNSaxisOffset & 0x8000)==0x8000)
	{
		if(CtParam.nCNSaxisOffset != 0x8000)
		{
			CtParam.nCNSaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_CNSAXIS_ALG_ADDR, (uint16_t)CtParam.nCNSaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "CT_CWEAXIS_ALIGN_ADDR(0x%02x) : [0x%04x]", 
	            ROM_CT_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR));
    CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
    if (CtParam.nCWEaxisOffset == (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset;
		else
		{
			CtParam.nCWEaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_CWEAXIS_ALG_ADDR, (uint16_t)CtParam.nCWEaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }
	else if((CtParam.nCWEaxisOffset & 0x8000)==0x8000)
	{
		if(CtParam.nCWEaxisOffset != 0x8000)
		{
			CtParam.nCWEaxisOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_CWEAXIS_ALG_ADDR, (uint16_t)CtParam.nCWEaxisOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}
	
	printUart(DBG_MSG_PC, "ROM_CT_RAXIS_PATIENT_ALG_ADDR(0x%02x) : [0x%04x]", 
            ROM_CT_RAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR));
	CtParam.nRaxisPatientOffset = EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
	if (CtParam.nRaxisPatientOffset == (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	CtParam.nRaxisPatientOffset = 0;
		else
		{
			CtParam.nRaxisPatientOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR, (uint16_t)CtParam.nRaxisPatientOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }
	else if((CtParam.nRaxisPatientOffset & 0x8000)==0x8000)
	{
		if(CtParam.nRaxisPatientOffset != 0x8000)
		{
			CtParam.nRaxisPatientOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR, (uint16_t)CtParam.nRaxisPatientOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	printUart(DBG_MSG_PC, "ROM_CT_PAXIS_PATIENT_ALG_ADDR(0x%02x) : [0x%04x]", 
            ROM_CT_PAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR));
	CtParam.nPaxisPatientOffset = EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
	if (CtParam.nPaxisPatientOffset == (signed short)0xffff)
    {
    	if(sysInfo.bIf0xffff==RESET)
        	CtParam.nPaxisPatientOffset = 0;
		else
		{
			CtParam.nPaxisPatientOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR, (uint16_t)CtParam.nPaxisPatientOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
    }
	else if((CtParam.nPaxisPatientOffset & 0x8000)==0x8000)
	{
		if(CtParam.nPaxisPatientOffset != 0x8000)
		{
			CtParam.nPaxisPatientOffset-=1;
			EEPRom_I2C_Write_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR, (uint16_t)CtParam.nPaxisPatientOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());
		}
	}

	if (sysInfo.model_id == MODEL_T2_CS)
    {
    	//191224 HWAN Ceph
		printUart(DBG_MSG_PC, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));
        CephParam.n2ndSpeedStepOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR);
        if (CephParam.n2ndSpeedStepOffset == (int16_t)0xffff)
        {
        	if(sysInfo.bIf0xffff==RESET)
            	CephParam.n2ndSpeedStepOffset = 0;
			else
			{
				CephParam.n2ndSpeedStepOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndSpeedStepOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
        }
		else if((CephParam.n2ndSpeedStepOffset & 0x8000)==0x8000)
		{
			if(CephParam.n2ndSpeedStepOffset != 0x8000)
			{
				CephParam.n2ndSpeedStepOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndSpeedStepOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
		}
        
		printUart(DBG_MSG_PC, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : [0x%04x]", 
	    	        ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
        CephParam.n2ndFastSpeedStepOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR);
        if (CephParam.n2ndFastSpeedStepOffset == (int16_t)0xffff)
        {
        	if(sysInfo.bIf0xffff==RESET)
            	CephParam.n2ndFastSpeedStepOffset = CephParam.n2ndSpeedStepOffset;
			else
			{
				CephParam.n2ndFastSpeedStepOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndFastSpeedStepOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
        }
		else if((CephParam.n2ndFastSpeedStepOffset & 0x8000)==0x8000)
		{
			if(CephParam.n2ndFastSpeedStepOffset != 0x8000)
			{
				CephParam.n2ndFastSpeedStepOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndFastSpeedStepOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
		}
		
		//190730 HWAN Ceph offset
		printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : [0x%04x]", 
		            ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));
		CephParam.n2ndColEndOffset = EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR);
		if (CephParam.n2ndColEndOffset == (signed short)0xffff)
		{
			if(sysInfo.bIf0xffff==RESET)
				CephParam.n2ndColEndOffset = 0;
			else
			{
				CephParam.n2ndColEndOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_END_ADDR, (uint16_t)CephParam.n2ndColEndOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
		}
		else if((CephParam.n2ndColEndOffset & 0x8000)==0x8000)
		{
			if(CephParam.n2ndColEndOffset != 0x8000)
			{
				CephParam.n2ndColEndOffset-=1;
				EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_END_ADDR, (uint16_t)CephParam.n2ndColEndOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());
			}
		}
    }	

	sysInfo.nFW_Com_Value=1;
	EEPRom_I2C_Write_Word(ROM_FW_COMPATIBILITY_ADDR, (uint16_t)sysInfo.nFW_Com_Value);
	IntTimer_Delay(1000);
	while(!IntTimer_GetStatus());
	
	printUart(DBG_MSG_PC, "ROM_FW_COMPATIBILITY_ADDR(0x%02x) : [0x%04x]", 
	            ROM_FW_COMPATIBILITY_ADDR, EEPRom_I2C_Read_Word(ROM_FW_COMPATIBILITY_ADDR));

	printUart(DBG_MSG_PC, "FW EEPROM DATA APPLY COMPLETE");

}


/**
* @ Function Name : EEPRom_LoadSysInfo
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_LoadSysInfo(void)
{	
	unsigned short ShowLog;
	
	ShowLog=EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
	
	if(ShowLog==0xffff || ((ShowLog&0x8000)==0x8000))
	{
		sysInfo.bShowLog=FALSE;
		sysInfo.bShowMotorLog=FALSE;
		sysInfo.bShowQueueLog=FALSE;

		EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR,0);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());   
	}
	else if((ShowLog&0x0001)==0x0001)
	{
		sysInfo.bShowLog=TRUE;
	}
	else
	{
		sysInfo.bShowLog=FALSE;
	}

	if((ShowLog&0x0010)==0x0010)
	{
		sysInfo.bShowMotorLog=TRUE;
	}
	else
	{
		sysInfo.bShowMotorLog=FALSE;
	}

	if((ShowLog&0x0100)==0x0100)
	{
		sysInfo.bShowQueueLog=TRUE;
	}
	else
	{
		sysInfo.bShowQueueLog=FALSE;
	}

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "ROM_FIRMWARE_SETTING_ADDR(0x%02x) : [0x%04x]", 
            ROM_FIRMWARE_SETTING_ADDR, EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR));
	}

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "\r\n");
	    printUart(DBG_MSG_PC, "ROM_BOARD_ID_ADDR(0x%02x) : [0x%04x]", 
	            ROM_BOARD_ID_ADDR, EEPRom_I2C_Read_Word(ROM_BOARD_ID_ADDR));
	}
	
    sysInfo.board_id = EEPRom_I2C_Read_Word(ROM_BOARD_ID_ADDR);
    if (sysInfo.board_id == 0xffff) {
        sysInfo.board_id = BOARD_REVISION_2_0;
    }

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "ROM_MODEL_ID_ADDR(0x%02x) : [0x%02x]", 
	            ROM_MODEL_ID_ADDR, EEPRom_I2C_Read_Byte(ROM_MODEL_ID_ADDR));
	}
    sysInfo.model_id = (modelNameId_t)EEPRom_I2C_Read_Byte(ROM_MODEL_ID_ADDR);
    if (sysInfo.model_id == 0xff) {
        sysInfo.model_id = MODEL_T2_CS;
    }
    
	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "ROM_CCAXIS_INIT_DIR_ADDR(0x%02x) : [0x%02x]", 
	            ROM_CCAXIS_INIT_DIR_ADDR, EEPRom_I2C_Read_Byte(ROM_CCAXIS_INIT_DIR_ADDR));
	}
	
    sysInfo.initDir_MotCC = (motorInitDir_t)EEPRom_I2C_Read_Byte(ROM_CCAXIS_INIT_DIR_ADDR);
    if (sysInfo.initDir_MotCC == 0xff) 
	{
        sysInfo.initDir_MotCC = INIT_DIR_REVERSE;	// 201204 HWAN for opinion of manufacturing department
        EEPRom_I2C_Write_Byte(ROM_CCAXIS_INIT_DIR_ADDR, (char)sysInfo.initDir_MotCC);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());   
    }

	if(sysInfo.bShowLog==TRUE)
	{
	    printUart(DBG_MSG_PC, "\r\n");
		printUart(DBG_MSG_PC, "ROM_FW_COMPATIBILITY_ADDR(0x%02x) : [0x%04x]", 
	            ROM_FW_COMPATIBILITY_ADDR, EEPRom_I2C_Read_Word(ROM_FW_COMPATIBILITY_ADDR));
	}
    sysInfo.nFW_Com_Value = (int16_t)EEPRom_I2C_Read_Word(ROM_FW_COMPATIBILITY_ADDR);
		
	if(sysInfo.nFW_Com_Value == (int16_t)0xffff)
	{
		sysInfo.nFW_Com_Value=0x0000;
	}
}

/**
* @ Function Name : EEPRom_EraseAll
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_EraseAll(void)
{
    unsigned short addr;

    for (addr=0xe0; addr <= kEpEndAddr; addr+=0x04) {
        EEPRom_EraseAddrDword((char)addr);

        printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%x", addr, EEPRom_I2C_Read_Dword(addr));
    }

    printUart(DBG_MSG_PC, "Erase All Completed.");
}

/**
* @ Function Name : EEPRom_EraseAddrByte
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_EraseAddrByte(char addr)
{
    printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%02x", addr, EEPRom_I2C_Read_Byte(addr));

    printUart(DBG_MSG_PC, "Erasing.......");
    
    EEPRom_I2C_Write_Byte(addr, 0xff);
    IntTimer_Delay(1000);
    while(!IntTimer_GetStatus());
    
    printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%02x", addr, EEPRom_I2C_Read_Byte(addr));
}

/**
* @ Function Name : EEPRom_EraseAddrWord
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_EraseAddrWord(char addr)
{
    printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", addr, EEPRom_I2C_Read_Word(addr));

    printUart(DBG_MSG_PC, "Erasing.......");
    
    EEPRom_I2C_Erase_Word(addr, 0xffff);
    IntTimer_Delay(1000);
    while(!IntTimer_GetStatus());
    
    printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", addr, EEPRom_I2C_Read_Word(addr));
}

/**
* @ Function Name : EEPRom_EraseAddrDword
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_EraseAddrDword(char addr)
{
    printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%08x", addr, EEPRom_I2C_Read_Dword(addr));

    printUart(DBG_MSG_PC, "Erasing.......");
    
    EEPRom_I2C_Write_Dword(addr, 0xffffffff);
    IntTimer_Delay(1000);
    while(!IntTimer_GetStatus());

    printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%08x", addr, EEPRom_I2C_Read_Dword(addr));
}

/**
* @ Function Name : EEPRom_ReadAll
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_ReadAll(void)
{
    unsigned short addr;

    for (addr=0x30; addr <= kEpEndAddr; addr+=2) {
        printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", addr, \
			EEPRom_I2C_Read_Word(addr));
    }
}

/**
* @ Function Name : EEPRom_SetAddress
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_SetAddress(char addr)
{
    romParam.addr = addr;
	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "address set to 0x%02x", romParam.addr);
	}
}

/**
* @ Function Name : EEPRom_SetWordData
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_SetWordData(unsigned short data)
{
    romParam.data = data;
	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "data set to 0x%04x", romParam.data);
	}
}

/**
* @ Function Name : EEPRom_StartWriteCmd
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_StartWriteCmd(void)
{
    printUart(DBG_MSG_PC, "(EEPROM) Write : Address(0x%02x), Data(0x%04x)", romParam.addr, romParam.data);
    
    EEPRom_I2C_Direct_Write_Word(romParam.addr, romParam.data);
    IntTimer_Delay(1000);
    while(!IntTimer_GetStatus());
}
/**
* @ Function Name : EEPRom_ShowOffset
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_ShowOffset(void)
{
    unsigned int cnt;

    printUart(DBG_MSG_PC, "\r\n\r\n");
    printUart(DBG_MSG_PC, "T2 EEPROM MAP");
    printUart(DBG_MSG_PC, "-----------------------------------------------------------");
    for (cnt = 0; cnt < ROM_MAP_COUNT; cnt++)
    {
    	if(cnt==7 || cnt ==8) continue;
        printUart(DBG_MSG_PC, " %s  | addr(0x%02x) = 0x%04x",
            map[cnt].name, map[cnt].addr, EEPRom_I2C_Read_Word(map[cnt].addr));
    }
    
    printUart(DBG_MSG_PC, "-----------------------------------------------------------");
}

/**
* @ Function Name : EEPRom_ShowMap
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_ShowMap(void)
{
    unsigned int cnt;

    printUart(DBG_MSG_PC, "\r\n\r\n");
    printUart(DBG_MSG_PC, "T2 EEPROM MAP");
    printUart(DBG_MSG_PC, "-----------------------------------------------------------");
    for (cnt = 0; cnt < ROM_MAP_COUNT; cnt++)
    {
        printUart(DBG_MSG_PC, " %s  | addr(0x%02x) = 0x%04x",
            map[cnt].name, map[cnt].addr, EEPRom_I2C_Read_Word(map[cnt].addr));
    }
    
    printUart(DBG_MSG_PC, "-----------------------------------------------------------");
}

/**
* @ Function Name : EEPRom_ModeLoop
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EEPRom_ModeLoop(void)
{
    printUart(DBG_MSG_PC, "EEPPROM Mode Started");

    while((!UART_SetRomParam()) && (CurCaptureMode != CAPTURE_CANCEL));

    printUart(DBG_MSG_PC, "EEPPROM Mode Terminated");
}
#endif /* USE_I2C_EEPROM */


/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

