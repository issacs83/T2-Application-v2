/*
*********************************************************************************************
* can.c :
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

/* Include files -------------------------------------------------------- */
#include "extern.h"
#include "can.h"
#include "system.h"
#include "error_code.h"
#include "tube.h"
#include "serial.h"


/* Private define ------------------------------------------------------ */
#define	CAN_CLOCK(can)					(can == CAN1) ? RCC_APB1Periph_CAN1 : RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2
#define	CAN_GPIO						GPIOB
#define	CAN_GPIO_CLOCK					RCC_AHB1Periph_GPIOB
#define	CAN_GPIO_PIN					GPIO_Pin_12 | GPIO_Pin_13
#define	CAN_GPIO_PinSource_Tx			GPIO_PinSource13
#define	CAN_GPIO_PinSource_Rx			GPIO_PinSource12
#define	CAN_GPIO_AF						GPIO_AF_CAN2
#define	CAN_RX_INTERRUPT				CAN2_RX0_IRQn

#define	CAN_BAUDRATE_1M							2
#define	CAN_BAUDRATE_500K						4

#define	CAN_BAUDRATE							CAN_BAUDRATE_500K

#define	CAN_TX_PENDING_COUNT 					0xffff6

#define	CAN_SUBB_RESPONSE 						0x1800
//#define	CAN_SUBB_RESPONSE_ERROR 0x1801 // System --> Collimator
#define	COLLI_RESPONSE_RESULT			    	0x1810	

#define	COLLI_VAL_TILTSTATUS_CT					0x00000010
#define	COLLI_VAL_TILTSTATUS_CEPH				0x00000020
#define	COLLI_VAL_TILTSTATUS_NONE				0x00000030
#define	COLLI_VAL_TILTSTATUS_NOSENSOR	    	0x00000040

#define	COLLI_VAL_RESP_COMPLETE					0x00000000
#define	COLLI_VAL_RESP_TIMEOUT					0x00000001
#define	COLLI_VAL_RESP_ERROR				    0x00000002
#define	COLLI_VAL_RESP_BUSY						0x00000003
#define	COLLI_VAL_RESP_UNKNOWN					0x00000004
#define	COLLI_VAL_RESP_BADCMD					0x00000005
#define	COLLI_VAL_RESP_BADVALUE		    		0x00000006

/* collimator : response error */
#define	VAL_ERROR_COLLI_TAXIS					0x0001
#define	VAL_ERROR_COLLI_BAXIS					0x0002
#define	VAL_ERROR_COLLI_LAXIS					0x0004
#define	VAL_ERROR_COLLI_RAXIS					0x0008
#define	VAL_ERROR_COLLI_FAXIS					0x0010
#define	VAL_ERROR_COLLI_HAXIS					0x0020
#define	VAL_ERROR_TILT_MOTOR					0x0040
#define	VAL_ERROR_TILT_SENSOR					0x0080
//#define VAL_ERROR_EEPROM						0x0100

#define	COLLI_RESPONSE_ERROR			    	0x1820	
#define	COLLI_RESPONSE_PLUS				    	0	
#define	COLLI_RESPONSE_MINUS			    	1	


/*
#define	CAN_TUBE_RESPONSE 				        	0x2600

#define CAN_TUBE_KV_STB_ERROR			        	0x2710
#define CAN_TUBE_mA_STB_ERROR 			        	0x2711
#define CAN_TUBE_FIL_STB_ERROR 			        	0x2712
#define CAN_TUBE_READY_FIL_ERROR 		        	0x2713
#define CAN_TUBE_KV_HIGH_OUTPUT 		        	0x2714
#define CAN_TUBE_KV_LOW_OUTPUT 		       	 		0x2715
#define CAN_TUBE_mA_HIGH_OUTPUT 		        	0x2716
#define CAN_TUBE_mA_LOW_OUTPUT 		        		0x2717
#define CAN_TUBE_KV_REF_ERROR 			        	0x2719
#define CAN_TUBE_mA_REF_ERROR 			        	0x2720
#define CAN_TUBE_TANK_TEMP_ERROR 		        	0x2721
#define CAN_TUBE_TANK_TEMP_OK_SIGNAL 	    		0x2722
#define CAN_TUBE_SEL_KV_mA_ERROR 		        	0x2723
#define CAN_TUBE_KV_REF_OUT_HI_OVER_ERROR 			0x2724
#define CAN_TUBE_KV_REF_OUT_LOW_OVER_ERROR 			0x2725
#define CAN_TUBE_mA_REF_OUT_HI_OVER_ERROR 			0x2726
#define CAN_TUBE_mA_REF_OUT_LOW_OVER_ERROR 			0x2727
#define CAN_TUBE_LIMIT_ERROR 			        	0x2728
#define CAN_TUBE_SEL_KV_DAC_ERROR 	        		0x2729
#define CAN_TUBE_SEL_mA_DAC_ERROR 	        		0x2730
#define CAN_TUBE_TANK_CON_ERROR 		    	    0x2731
#define CAN_TUBE_TANK_TEMP_SET_ERROR	        	0x2732
#define CAN_TUBE_CMD_ERROR          	   		    0x2733
//200324 HWAN
#define CAN_TUBE_kV_HIGH_Output_WARNING   			0x2750				//WARNING_102
#define CAN_TUBE_kV_LOW_Output_WARNING   			0x2751				//WARNING_103
#define CAN_TUBE_kV_Ref_Out_HI_OVER_WARNING   		0x2752				//WARNING_105
#define CAN_TUBE_kV_Ref_Out_LOW_OVER_WARNING   		0x2753				//WARNING_106
#define CAN_TUBE_mA_HIGH_Output_WARNING   			0x2754				//WARNING_202
#define CAN_TUBE_mA_LOW_Output_WARNING   			0x2755				//WARNING_203
#define CAN_TUBE_mA_Ref_Out_HI_OVER_WARNING   		0x2756				//WARNING_205
#define CAN_TUBE_mA_Ref_Out_LOW_OVER_WARNING   		0x2757				//WARNING_206 */

#define CAN_QUEUE_SIZE		20

#define CAN_COLLI_TIMEOUT 5000


/* Private typedef ----------------------------------------------------- */
typedef struct
{
	uint32_t	ExtId;
	uint16_t	Cmd;
	uint32_t	Value;
	uint16_t	Debug;
} CAN_MsgTypedef;

typedef struct
{
	CAN_MsgTypedef	Message[CAN_QUEUE_SIZE];
	uint32_t	Head;
	uint32_t	Tail;
} CAN_QueueTypedef;


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
CanTxMsg CAN_TxMessage;
CanRxMsg CAN_RxMessage;
CAN_MsgTypedef CAN_RspMessage;
CAN_QueueTypedef CAN_Queue;
CAN_QueueTypedef CollimCAN_Queue;
static uint32_t g_nMultiplier;

const tubeError_t tubeErrorTable[] = {
    { CAN_TUBE_KV_STB_ERROR, ERR_CODE_KV_STB_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_mA_STB_ERROR, ERR_CODE_MA_STB_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_FIL_STB_ERROR, ERR_CODE_FIL_STB_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_READY_FIL_ERROR, ERR_CODE_READY_FIL_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_KV_HIGH_OUTPUT, ERR_CODE_KV_HIGH_OUTPUT, ERROR_LEVEL_3 },
    { CAN_TUBE_KV_LOW_OUTPUT, ERR_CODE_KV_LOW_OUTPUT, ERROR_LEVEL_3 },
    { CAN_TUBE_mA_HIGH_OUTPUT, ERR_CODE_MA_HIGH_OUTPUT, ERROR_LEVEL_3 },
    { CAN_TUBE_mA_LOW_OUTPUT, ERR_CODE_MA_LOW_OUTPUT, ERROR_LEVEL_3 },
    { CAN_TUBE_KV_REF_ERROR, ERR_CODE_KV_REF_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_mA_REF_ERROR, ERR_CODE_MA_REF_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_TANK_TEMP_ERROR, ERR_CODE_TANK_TEMP_OVER, ERROR_LEVEL_3 },
    { CAN_TUBE_TANK_TEMP_OK_SIGNAL, ERR_CODE_TANK_TEMP_OK_SIG, ERROR_LEVEL_1 },
    { CAN_TUBE_SEL_KV_mA_ERROR, ERR_CODE_SEL_KV_MA_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_KV_REF_OUT_HI_OVER_ERROR, ERR_CODE_KV_REF_OUT_HI_OVER, ERROR_LEVEL_3 },
    { CAN_TUBE_KV_REF_OUT_LOW_OVER_ERROR, ERR_CODE_KV_REF_OUT_LOW_OVER, ERROR_LEVEL_3 },
    { CAN_TUBE_mA_REF_OUT_HI_OVER_ERROR, ERR_CODE_MA_REF_OUT_HI_OVER, ERROR_LEVEL_3 },
    { CAN_TUBE_mA_REF_OUT_LOW_OVER_ERROR, ERR_CODE_MA_REF_OUT_LOW_OVER, ERROR_LEVEL_3 },
    { CAN_TUBE_LIMIT_ERROR, ERR_CODE_LIMIT_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_SEL_KV_DAC_ERROR, ERR_CODE_SEL_KV_DAC_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_SEL_mA_DAC_ERROR, ERR_CODE_SEL_MA_DAC_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_TANK_CON_ERROR, ERR_CODE_TANK_CON_ERROR, ERROR_LEVEL_3 },
    { CAN_TUBE_TANK_TEMP_SET_ERROR, ERR_CODE_TANK_TEMP_SET_ERROR, ERROR_LEVEL_1 },
    { CAN_TUBE_CMD_ERROR, ERR_CODE_CMD_ERROR, ERROR_LEVEL_1 },
    { CAN_Tank_FAN_Temp_Setting_ERROR, ERR_CODE_TANK_FAN_TEMP_SET_ERROR, ERROR_LEVEL_1 },
};
#define TABLE_CNT (sizeof(tubeErrorTable) / sizeof(tubeError_t))


/* Private function prototypes ------------------------------------------*/
static void CAN_Delay_Init(void);
static bool CAN_TransmitMessage(bool isGen, unsigned short cmd);
static bool CAN_ProcTxMessage(uint32_t extid, uint16_t cmd, uint32_t value, uint16_t debug);
static void CAN_QueueInit(CAN_QueueTypedef* queue);
static uint32_t CAN_QueueCnt(CAN_QueueTypedef* queue);
static bool CAN_Enqueue(CAN_QueueTypedef* queue, CAN_MsgTypedef Message);
static CAN_MsgTypedef CAN_Dequeue(CAN_QueueTypedef* queue);
static void CAN_GeneratorErrorCheck(unsigned short error);

  
/* Private functions -------------------------------------------------- */
/**
* @ Function Name : CAN_Delay_Init
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CAN_Delay_Init(void) 
{
    RCC_ClocksTypeDef RCC_Clocks;

    /* Get system clocks */
    RCC_GetClocksFreq(&RCC_Clocks);

    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */

    g_nMultiplier = (RCC_Clocks.HCLK_Frequency / 4000000) / 7;
}

/**
* @ Function Name : CAN_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void CAN_Config(void)
{	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(CAN_CLOCK(CAN), ENABLE);
	
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(CAN_GPIO_CLOCK, ENABLE);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = CAN_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(CAN_GPIO, &GPIO_InitStructure);	
	
	/* Connect CAN pins to AFx */
	GPIO_PinAFConfig(CAN_GPIO, CAN_GPIO_PinSource_Tx,	 CAN_GPIO_AF);
	GPIO_PinAFConfig(CAN_GPIO, CAN_GPIO_PinSource_Rx,	 CAN_GPIO_AF);
	
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

	/* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitStructure.CAN_Prescaler = CAN_BAUDRATE;	
	/* CANx configuration */
	CAN_Init(CAN, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Transmit Structure preparation */
	CAN_TxMessage.StdId = CAN_STD_ID;
    //CAN_TxMessage.ExtId = CAN_EXT_ID_SYSTEM_TO_TUBE;
    //CAN_TxMessage.ExtId = CAN_EXT_ID_SYSTEM_TO_SUBB;
	CAN_TxMessage.RTR = CAN_RTR_DATA;
	CAN_TxMessage.IDE = CAN_ID_EXT;
	CAN_TxMessage.DLC = CAN_FRAME_LENGTH_BYTE;		

	/* CAN FIFO0 message pending interrupt enable */ 
	CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE);
	
	/* Enable the CAN2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_INTERRUPT;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Queue Initialize */
	CAN_QueueInit(&CAN_Queue);
    CAN_QueueInit(&CollimCAN_Queue);

    CAN_Delay_Init();

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "Can Config Done");
	}
}


/**
* @ Function Name : CAN_TransmitMessage
* @ Desc : 
* @ Param : 
* @ Return :
*/
static bool CAN_TransmitMessage(bool isGen, unsigned short cmd)
{
	uint8_t output=0;
    unsigned int nCnt = 0;
    unsigned char txMailbox = CAN_Transmit(CAN, &CAN_TxMessage);
    while(CAN_TransmitStatus(CAN, txMailbox) != CANTXOK)
    {
        if (nCnt++ >= CAN_TX_PENDING_COUNT)
        {
            if (isGen)
            {
                printUart(DBG_MSG_PC, ERR_CODE_TUBE_TX_MSG_ERROR" : 0x%04x", cmd);
            }
            else
            {
                printUart(DBG_MSG_PC, ERR_CODE_COL_TX_MSG_ERROR" : 0x%04x", cmd);
            }
			output=CAN_TransmitStatus(CAN, txMailbox);
			if(output==CANTXOK)
				printUart(DBG_MSG_PC, "CANTXOK : %d",txMailbox);
			else if(output==CANTXFAILED)
				printUart(DBG_MSG_PC, "CANTXFAILED : %d",txMailbox);
			else if(output==CANTXPENDING)
				printUart(DBG_MSG_PC, "CANTXPENDING : %d",txMailbox);
			else if(output==CAN_NO_MB)
				printUart(DBG_MSG_PC, "CAN_NO_MB : %d",txMailbox);
            return FALSE;
        }
    }

    return TRUE;
}

/**
* @ Function Name : CAN_ProcTxMessage
* @ Desc : 
* @ Param : 
* @ Return :
*/
static bool CAN_ProcTxMessage(uint32_t extid, uint16_t cmd, uint32_t value, uint16_t debug)
{
    uint16_t high2byte = value >> 16;
    uint16_t low2byte = value & 0xffff;
    uint16_t response = CAN_SUBB_RESPONSE;
    bool gen = FALSE;
    bool ret = FALSE;

    if (extid == CAN_EXT_ID_SYSTEM_TO_SUBB)
    {
        //
    }
    else if(extid ==  CAN_EXT_ID_SYSTEM_TO_TUBE)
    {
        if (cmd == CAN_TUBE_KV_SET)
        {
            if (!(value <= MAX_TUBE_KV && value >= MIN_TUBE_KV))
            {
                printUart(DBG_MSG_PC, "%d kV is range error", value);
                return ret;
            }
        }
        else if (cmd == CAN_TUBE_mA_SET)
        {
            if (!(value <= MAX_TUBE_mA && value >= MIN_TUBE_mA))
            {
                printUart(DBG_MSG_PC, "%d mA is range error", value);            
                return ret;
            }
        }
        else if (cmd == CAN_TUBE_TANK_TEMP_SET)
        {
            if (!(value <= MAX_TANK_TEMP && value >= MIN_TANK_TEMP))
            {
                printUart(DBG_MSG_PC, "%d Temp is range error", value);
                return ret;
            }
        }

        response = CAN_TUBE_RESPONSE;
        gen = TRUE;
    }
    else
    {
        printUart(DBG_MSG_PC, "Unsupported ExtID : 0x%08x", extid);
        return ret;
    }

    CAN_TxMessage.ExtId = extid;
    CAN_TxMessage.Data[0] = cmd >> 8;
    CAN_TxMessage.Data[1] = cmd & 0xff;
    CAN_TxMessage.Data[2] = high2byte >> 8;
    CAN_TxMessage.Data[3] = high2byte & 0xff;
    CAN_TxMessage.Data[4] = low2byte >> 8;
    CAN_TxMessage.Data[5] = low2byte & 0xff;
    CAN_TxMessage.Data[6] = debug >> 8;
    CAN_TxMessage.Data[7] = debug & 0xff;

    CAN_RspMessage.Value = value;
    CAN_RspMessage.Cmd = response;

    ret = CAN_TransmitMessage(gen, cmd);

    return ret;
}

/**
* @ Function Name : CAN_Collimator_SendMessage
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value, uint16_t debug, uint32_t timeOut, uint8_t retryCnt)
{
	CAN_MsgTypedef canMsg;
    uint32_t nQueueCnt = 0;

    gMachStat.bColliCommRxErr = TRUE;

    if (!CAN_ProcTxMessage(CAN_EXT_ID_SYSTEM_TO_SUBB, cmd, value, debug)) 
	{
        return FALSE;
    }
    
    if (timeOut) 
	{
        while(1) 
        {
            /* Multiply millis with multipler */
            /* Substract 10 */
            uint32_t millis = 1000 * CAN_COLLI_TIMEOUT * g_nMultiplier - 10;
            /* 4 cycles for one loop */
            while(millis--)
            {
                nQueueCnt = CAN_QueueCnt(&CollimCAN_Queue);
                if (nQueueCnt)
					goto Parsing;
            }

            if (!retryCnt) 
			{
				printUart(DBG_MSG_PC, ERR_CODE_COLLI_RETRY_MSG_ERROR" : 0x%04x", cmd);
                break;
            }
            
            printUart(DBG_MSG_PC, "COLLIMATOR MSG(0x%04x) : RETRY(%d)", cmd, retryCnt);

            if (!CAN_TransmitMessage(FALSE, cmd)) 
			{
                return FALSE;
            }
            
            retryCnt--;
        }
    }
    else
    {
    	while(!CAN_QueueCnt(&CollimCAN_Queue));
        goto Parsing;
    }
    
    return FALSE;

Parsing:

    gMachStat.bColliCommRxErr = FALSE;

    canMsg = CAN_Dequeue(&CollimCAN_Queue);

	if(sysInfo.bShowLog==TRUE && sysInfo.bShowQueueLog==TRUE)
		printUart(DBG_MSG_PC, "COLLIMATOR MSG(0x%04x) : 0x%04x : %d : %d", cmd, canMsg.Cmd, canMsg.Value, canMsg.Debug);
	
	if(canMsg.Cmd == cmd)
	{
		if (canMsg.Debug == CAN_SUBB_RESPONSE)
	    {
	        if (cmd == CMD_TILT_STATUS)
	        {
	            char str[64] = {0,};
	            
	            sprintf(str, "Tilting Motor Check :");
	            if (canMsg.Value == COLLI_VAL_TILTSTATUS_CT)
	            {
	                sprintf(&str[21], " CT (TILT OFF)");
	            }
	            else if (canMsg.Value == COLLI_VAL_TILTSTATUS_CEPH)
	            {
	                sprintf(&str[21], " CEPH (TILT ON)");
	            }
	            else if (canMsg.Value == COLLI_VAL_TILTSTATUS_NONE)
	            {
	                sprintf(&str[21], " NONE (TILT NONFIXED)");
	            }
	            else 
	            {
	                sprintf(&str[21], " Unknown Value (need to check ErrorCode)");
	                gMachStat.bColliErr = TRUE;
	            }
	            
	            printUart(DBG_MSG_PC, str);
	        }
	        else if (cmd == CMD_STATUS_CHECK)
	        {
	            printUart(DBG_MSG_PC, "Clear ErrCode for Collimator");
	            gMachStat.bColliErr = FALSE;
	        }
			else if(cmd == CMD_COLLI_CALIBRATION_POSITION)
			{				
				char string[64] = {0,};
				sprintf(string, "Collimator POSITION MOVE COMPLETE");
				printUart(DBG_MSG_PC, "%s Speed : %d", string, canMsg.Value);
			}
			else if(cmd == CMD_COLLI_CALIBRATION_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator STOP COMPLETE");
			}
			else if(cmd == CMD_COLLI_MOTOR_ALL_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator ALL STOPED");
			}
			else if(cmd == CMD_COLLI_MOTOR_T_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator TOP STOPED");
			}
			else if(cmd == CMD_COLLI_MOTOR_B_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator BOTTOM STOPED");
			}
			else if(cmd == CMD_COLLI_MOTOR_L_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator LEFT STOPED");
			}
			else if(cmd == CMD_COLLI_MOTOR_R_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator RIGHT STOPED");
			}
			else if(cmd == CMD_COLLI_MOTOR_F_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator FILTER STOPED");
			}
			else if(cmd == CMD_COLLI_MOTOR_H_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator LASER STOPED");
			}
	    }
	    else if (canMsg.Debug == COLLI_RESPONSE_RESULT)
	    {
			if (cmd == CMD_VERSION_CHECK)
			{
				printUart(DBG_MSG_PC, "Collimator F/W Ver: %x.%x.%x", (uint8_t)(canMsg.Value >> 16), \
	                        (uint8_t)(canMsg.Value >> 8), (uint8_t)canMsg.Value);
			}            
			else if (cmd == CMD_BUILD_CHECK)
			{
				printUart(DBG_MSG_PC, "Collimator F/W Build : %x.%x.%x", (uint8_t)(canMsg.Value >> 16), \
	                        (uint8_t)(canMsg.Value >> 8), (uint8_t)canMsg.Value);
			}
			else if(cmd == CMD_BUILD_SUB_CHECK)
			{
				printUart(DBG_MSG_PC, "Collimator F/W SUB Build : %c", (char)canMsg.Value);
			}
			else if(cmd == CMD_COLLI_MOTOR_ALL_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator ALL Motor Status STOP");
			}
			else if(cmd == CMD_COLLI_MOTOR_ALL_RUN)
			{
				printUart(DBG_MSG_PC, "Collimator ALL Motor Move Complete");
			}
			else if(cmd == CMD_COLLI_MOTOR_T_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator Top Motor Status STOP");
			}
			else if(cmd == CMD_COLLI_MOTOR_B_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator Bottom Motor Status STOP");
			}
			else if(cmd == CMD_COLLI_MOTOR_L_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator Left Motor Status STOP");
			}
			else if(cmd == CMD_COLLI_MOTOR_R_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator Right Motor Status STOP");
			}
			else if(cmd == CMD_COLLI_MOTOR_F_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator Filter Motor Status STOP");
			}
			else if(cmd == CMD_COLLI_MOTOR_H_STATUS_STOP)
			{
				printUart(DBG_MSG_PC, "Collimator Laser Motor Status STOP");
			}
	        else // SendInfo_Offset
	        {
	            
	        }
	    }    	
		else if (canMsg.Debug == COLLI_RESPONSE_PLUS || canMsg.Debug == COLLI_RESPONSE_MINUS)
		{
			char string[64] = {0,};

            if (cmd == CMD_REQ_OFFSET_PANO_T)
                sprintf(string, "COLI_PANO_T_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_PANO_B)
                sprintf(string, "COLI_PANO_B_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_PANO_L)
                sprintf(string, "COLI_PANO_L_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_PANO_R)
                sprintf(string, "COLI_PANO_R_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CT_T)
                sprintf(string, "COLI__CT__T_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CT_B)
                sprintf(string, "COLI__CT__B_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CT_L)
                sprintf(string, "COLI__CT__L_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CT_R)
                sprintf(string, "COLI__CT__R_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CEPHSCAN_T)
                sprintf(string, "COLI_SCAN_T_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CEPHSCAN_B)
                sprintf(string, "COLI_SCAN_B_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CEPHSCAN_L)
                sprintf(string, "COLI_SCAN_L_OFFSET :");
            else if (cmd == CMD_REQ_OFFSET_CEPHSCAN_R)
                sprintf(string, "COLI_SCAN_R_OFFSET :");
			else if (cmd == CMD_REQ_OFFSET_HBEAM_LASER)
				sprintf(string, "COLI__BEAM__OFFSET :");
			else if (cmd == CMD_OFFSET_PANO_T)
                sprintf(string, "COLI_PANO_T___SENT :");
            else if (cmd == CMD_OFFSET_PANO_B)
                sprintf(string, "COLI_PANO_B___SENT :");
            else if (cmd == CMD_OFFSET_PANO_L)
                sprintf(string, "COLI_PANO_L___SENT :");
            else if (cmd == CMD_OFFSET_PANO_R)
                sprintf(string, "COLI_PANO_R___SENT :");
            else if (cmd == CMD_OFFSET_CT_T)
                sprintf(string, "COLI__CT__T___SENT :");
            else if (cmd == CMD_OFFSET_CT_B)
                sprintf(string, "COLI__CT__B___SENT :");
            else if (cmd == CMD_OFFSET_CT_L)
                sprintf(string, "COLI__CT__L___SENT :");
            else if (cmd == CMD_OFFSET_CT_R)
                sprintf(string, "COLI__CT__R___SENT :");
            else if (cmd == CMD_OFFSET_CEPHSCAN_T)
                sprintf(string, "COLI_SCAN_T___SENT :");
            else if (cmd == CMD_OFFSET_CEPHSCAN_B)
                sprintf(string, "COLI_SCAN_B___SENT :");
            else if (cmd == CMD_OFFSET_CEPHSCAN_L)
                sprintf(string, "COLI_SCAN_L___SENT :");
            else if (cmd == CMD_OFFSET_CEPHSCAN_R)
                sprintf(string, "COLI_SCAN_R___SENT :");
			else if (cmd == CMD_OFFSET_HBEAM_LASER)
				sprintf(string, "COLI__BEAM____SENT :");
			else if (cmd == CMD_COLLI_CALIBRATION_TOP_POSITION)
				sprintf(string, "COLI__TOP_____POSI :");
			else if (cmd == CMD_COLLI_CALIBRATION_BOTTOM_POSITION)
				sprintf(string, "COLI__BOTTOM__POSI :");
			else if (cmd == CMD_COLLI_CALIBRATION_LEFT_POSITION)
				sprintf(string, "COLI__LEFT____POSI :");
			else if (cmd == CMD_COLLI_CALIBRATION_RIGHT_POSITION)
				sprintf(string, "COLI__RIGHT___POSI :");
			else if (cmd == CMD_COLLI_CALIBRATION_PANO_TOP_STOP)
				sprintf(string, "COLI__TOP_____STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_PANO_BOTTOM_STOP)
				sprintf(string, "COLI__BOTTOM__STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_PANO_LEFT_STOP)
				sprintf(string, "COLI__LEFT____STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_PANO_RIGHT_STOP)
				sprintf(string, "COLI__RIGHT___STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_CT_TOP_STOP)
				sprintf(string, "COLI__TOP_____STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_CT_BOTTOM_STOP)
				sprintf(string, "COLI__BOTTOM__STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_CT_LEFT_STOP)
				sprintf(string, "COLI__LEFT____STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_CT_RIGHT_STOP)
				sprintf(string, "COLI__RIGHT___STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_SCAN_TOP_STOP)
				sprintf(string, "COLI__TOP_____STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_SCAN_BOTTOM_STOP)
				sprintf(string, "COLI__BOTTOM__STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_SCAN_LEFT_STOP)
				sprintf(string, "COLI__LEFT____STOP :");
			else if (cmd == CMD_COLLI_CALIBRATION_SCAN_RIGHT_STOP)
				sprintf(string, "COLI__RIGHT___STOP :");
			else if (cmd == CMD_COLLI_MOTOR_T_RUN)
				sprintf(string, "COLI__TOP____MOVED :");
			else if (cmd == CMD_COLLI_MOTOR_B_RUN)
				sprintf(string, "COLI__BOTTOM_MOVED :");
			else if (cmd == CMD_COLLI_MOTOR_L_RUN)
				sprintf(string, "COLI__LEFT___MOVED :");
			else if (cmd == CMD_COLLI_MOTOR_R_RUN)
				sprintf(string, "COLI__RIGHT__MOVED :");
			else if (cmd == CMD_COLLI_MOTOR_F_RUN)
				sprintf(string, "COLI__FILTER_MOVED :");
			else if (cmd == CMD_COLLI_MOTOR_H_RUN)
				sprintf(string, "COLI__LASER__MOVED :");
            
            //jehun - 20200720
//            sprintf(&string[20], canMsg.Debug ? " +%03d":" -%03d", canMsg.Value);
//            printUart(DBG_MSG_PC, string);
            if(canMsg.Debug) {
            	printUart(DBG_MSG_PC, "%s +%03d", string, canMsg.Value);
            }
            else {
            	printUart(DBG_MSG_PC, "%s -%03d", string, canMsg.Value);
            }


            //printUart(DBG_MSG_PC, canMsg.Debug ? " +%03d":" -%03d", canMsg.Value);
		}
	    else if (canMsg.Debug == COLLI_RESPONSE_ERROR)
	    {
	        if (cmd == CMD_STATUS_CHECK)
	        {
	            if (canMsg.Value & VAL_ERROR_COLLI_TAXIS)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_T_AXIS_ERROR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2) gMachStat.ErrorLevel = ERROR_LEVEL_2;
	            }
	            if (canMsg.Value & VAL_ERROR_COLLI_BAXIS)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_B_AXIS_ERROR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2) gMachStat.ErrorLevel = ERROR_LEVEL_2;
	            }
	            if (canMsg.Value & VAL_ERROR_COLLI_LAXIS)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_L_AXIS_ERROR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2) gMachStat.ErrorLevel = ERROR_LEVEL_2;
	            }
	            if (canMsg.Value & VAL_ERROR_COLLI_RAXIS)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_R_AXIS_ERROR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2) gMachStat.ErrorLevel = ERROR_LEVEL_2;
	            }
	            if (canMsg.Value & VAL_ERROR_COLLI_FAXIS)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_F_AXIS_ERROR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2) gMachStat.ErrorLevel = ERROR_LEVEL_2;
	            }
	            if (canMsg.Value & VAL_ERROR_COLLI_HAXIS)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_H_AXIS_ERROR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2 && CurCaptureMode != CAPTURE_CT) gMachStat.ErrorLevel = ERROR_LEVEL_2;
					else if(gMachStat.ErrorLevel==ERROR_LEVEL_1 && CurCaptureMode == CAPTURE_CT) gMachStat.ErrorLevel = ERROR_LEVEL_1;
	            }
	            if (canMsg.Value & VAL_ERROR_TILT_MOTOR)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_TILT_MOTOR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_3) gMachStat.ErrorLevel = ERROR_LEVEL_3;
	            }
	            if (canMsg.Value & VAL_ERROR_TILT_SENSOR)
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_TILT_NO_SENSOR);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_3) gMachStat.ErrorLevel = ERROR_LEVEL_3;
	            }
	        }
			else if(canMsg.Cmd == CMD_COLLI_CALIBRATION_POSITION)
			{
				printUart(DBG_MSG_PC, ERR_CODE_COL_OVER_VALUE_ERROR);					
				printUart(DBG_MSG_PC, "Send Value Error - Over 4608");
				printUart(DBG_MSG_PC, "%x %x", canMsg.Cmd, canMsg.Value);
			}
			else if(canMsg.Cmd == CMD_COLLI_CALIBRATION_TOP_POSITION)
			{
				printUart(DBG_MSG_PC, ERR_CODE_COL_OVER_VALUE_ERROR);					
				printUart(DBG_MSG_PC, "Motor Top Send Value Error - Over 4608");
				printUart(DBG_MSG_PC, "%x %x", canMsg.Cmd, canMsg.Value);
			}
			else if(canMsg.Cmd == CMD_COLLI_CALIBRATION_BOTTOM_POSITION)
			{
				printUart(DBG_MSG_PC, ERR_CODE_COL_OVER_VALUE_ERROR);					
				printUart(DBG_MSG_PC, "Motor Bottom Send Value Error - Over 4608");
				printUart(DBG_MSG_PC, "%x %x", canMsg.Cmd, canMsg.Value);
			}
			else if(canMsg.Cmd == CMD_COLLI_CALIBRATION_LEFT_POSITION)
			{
				printUart(DBG_MSG_PC, ERR_CODE_COL_OVER_VALUE_ERROR);					
				printUart(DBG_MSG_PC, "Motor Left Send Value Error - Over 4608");
				printUart(DBG_MSG_PC, "%x %x", canMsg.Cmd, canMsg.Value);
			}
			else if(canMsg.Cmd == CMD_COLLI_CALIBRATION_RIGHT_POSITION)
			{
				printUart(DBG_MSG_PC, ERR_CODE_COL_OVER_VALUE_ERROR);					
				printUart(DBG_MSG_PC, "Motor Right Send Value Error - Over 4608");
				printUart(DBG_MSG_PC, "%x %x", canMsg.Cmd, canMsg.Value);
			}
	        else
	        {
	        	if (canMsg.Value == COLLI_VAL_RESP_TIMEOUT)
	        	{
	                printUart(DBG_MSG_PC, ERR_CODE_COL_TILT_TIMEOUT);
	                if(gMachStat.ErrorLevel<ERROR_LEVEL_2) gMachStat.ErrorLevel = ERROR_LEVEL_2;
	        	}
				else if(canMsg.Value == COLLI_VAL_RESP_BADVALUE)
				{
					printUart(DBG_MSG_PC, ERR_CODE_COL_OVER_VALUE_ERROR);					
					printUart(DBG_MSG_PC, "Send Value Error - Over 4608");
					if(gMachStat.ErrorLevel==ERROR_LEVEL_1) gMachStat.ErrorLevel = ERROR_LEVEL_1;
				}
	            else
	            {
	            	printUart(DBG_MSG_PC, "Received Error Value : %d", canMsg.Value);
	                if(gMachStat.ErrorLevel==ERROR_LEVEL_1) gMachStat.ErrorLevel = ERROR_LEVEL_1;
	            }
	        }
	        gMachStat.bColliErr = TRUE;

	        return FALSE;
	    }
	    else
	    {
	    	printUart(DBG_MSG_PC, ERR_CODE_COL_RESP_ERROR" : %x %x %x %x", \
	                        canMsg.ExtId, canMsg.Cmd, canMsg.Value, canMsg.Debug);
			return FALSE;
			
	    }
	}
	else
	{			
		if(gMachStat.uiColliMatchCnt < 1)
		{
			gMachStat.uiColliMatchCnt++;
			printUart(DBG_MSG_PC, ERR_CODE_COL_NOT_MATCH_ERROR"%x : %x %x %x %x", cmd, canMsg.ExtId, canMsg.Cmd, canMsg.Value, canMsg.Debug);
			CAN_QueueInit(&CollimCAN_Queue);
			if(CAN_Collimator_SendMessage(cmd,value,debug,timeOut,retryCnt)==FALSE)
			{
				return FALSE;
			}
		}
		else
		{
			CAN_QueueInit(&CollimCAN_Queue);
			printUart(DBG_MSG_PC, ERR_CODE_COL_NOT_MATCH_MAX_ERROR"%x : %x %x %x %x", cmd,canMsg.ExtId, canMsg.Cmd, canMsg.Value, canMsg.Debug);
			return FALSE;
		}
	}
	gMachStat.uiColliMatchCnt=0;    
	

    return TRUE;
}

/**
* @ Function Name : CAN_SendMessage
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool CAN_SendMessage(uint16_t cmd, uint32_t value, 
                        uint16_t debug, uint32_t timeOut, uint8_t retryCnt)
{
	CAN_MsgTypedef canMsg;
    uint32_t nQueueCnt = 0;

    gMachStat.bTubeCommRxErr = TRUE;
    //gMachStat.bTubeErr = FALSE;

    if (!CAN_ProcTxMessage(CAN_EXT_ID_SYSTEM_TO_TUBE, cmd, value, debug)) {
        return FALSE;
    }

    if (timeOut) {
        while(1) 
        {
            /* Multiply millis with multipler */
            /* Substract 10 */
            uint32_t millis = 1000 * timeOut * g_nMultiplier - 10;
            /* 4 cycles for one loop */
            while(millis--)
            {
                nQueueCnt = CAN_QueueCnt(&CAN_Queue);
                if (nQueueCnt) goto Parsing;
            }

            if (!retryCnt) 
			{
				printUart(DBG_MSG_PC, ERR_CODE_TUBE_RETRY_MSG_ERROR" : 0x%04x", cmd);
                break;
            }
            
            printUart(DBG_MSG_PC, "GENERATOR MSG(0x%04x) : RETRY(%d)", cmd, retryCnt);

            if (!CAN_TransmitMessage(TRUE, cmd)) {
                return FALSE;
            }
            
            retryCnt--;
        }
    }
    else
    {
        while(!CAN_QueueCnt(&CAN_Queue));
        goto Parsing;
    }

    return FALSE;
    
Parsing:

    gMachStat.bTubeCommRxErr = FALSE;
    
    canMsg = CAN_Dequeue(&CAN_Queue);
	
	if(sysInfo.bShowLog==TRUE && sysInfo.bShowQueueLog==TRUE)
		printUart(DBG_MSG_PC, "GENERATOR MSG(0x%04x) : 0x%04x", cmd, canMsg.Cmd);  
	
    if (canMsg.Cmd == cmd)
    {
    
        switch(cmd)
        {
        case CAN_TUBE_VER_CHECK:
			printUart(DBG_MSG_PC, "Generator F/W Version : %d.%d.%d", \
			        (uint8_t)((canMsg.Value >> 8) / 100), \
			        (uint8_t)((canMsg.Value >> 8) % 10), \
			        (uint8_t)canMsg.Value);
            break;
		case CAN_Sub_Version_Check:
			printUart(DBG_MSG_PC, "Generator F/W SUB Version : %c", (char)canMsg.Value);
			break;
        case CAN_TUBE_COUNT_RESET:
			printUart(DBG_MSG_PC, "Tube Usage Count Reset Complete");
            printUart(DBG_MSG_PC, "Tube Usage Count : %d", canMsg.Value);
            break;    
        case CAN_TUBE_COUNT_CHECK:
            printUart(DBG_MSG_PC, "Tube Usage Count : %d", canMsg.Value);
			if(gMachStat.bTubeCountFlag==SET) gMachStat.nTubeCount = canMsg.Value;
			else gMachStat.nTubeOldCount = canMsg.Value;
            break;

        case CAN_TUBE_CONTI_MODE_SET:
            printUart(DBG_MSG_PC, "Set Tank Continues Mode");
            break;

        case CAN_TUBE_PULSE_MODE_SET:
            printUart(DBG_MSG_PC, "Set Tank Pulsed Mode");
            break;

        case CAN_TUBE_TANK_TEMP_SET:
            printUart(DBG_MSG_PC, "Set Threshold Temperature : %d", (uint8_t)canMsg.Value);
            break;

        case CAN_TUBE_TANK_TEMP_SET_READ:
            printUart(DBG_MSG_PC, "Threshold Temperature : %d", (uint8_t)canMsg.Value);
            break;

		case CAN_TUBE_KV_SET:
            printUart(DBG_MSG_PC, "Set Tube kV : %d", (uint8_t)canMsg.Value);
            break;
			
        case CAN_TUBE_KV_REF_READ:
            printUart(DBG_MSG_PC, "Tube KV Reference : %d", (uint8_t)(canMsg.Value >> 8));
            break;
            
        case CAN_TUBE_KV_FB_READ:
            printUart(DBG_MSG_PC, "Tube KV Feedback : %d", (uint8_t)(canMsg.Value >> 8));
            break;
			
		case CAN_TUBE_mA_SET:
            printUart(DBG_MSG_PC, "Set Tube mA : %d.%d", (uint8_t)canMsg.Value/10,(uint8_t)canMsg.Value%10);
            break;
			
        case CAN_TUBE_mA_REF_READ:
            printUart(DBG_MSG_PC, "Tube mA Reference : %d", (uint8_t)(canMsg.Value >> 8));
            break;
            
        case CAN_TUBE_mA_FB_READ:
            printUart(DBG_MSG_PC, "Tube mA Feedback : %d", (uint8_t)(canMsg.Value >> 8));
            break;
            
		case CAN_TUBE_READY_ON:
            printUart(DBG_MSG_PC, "Tube Ready");
            break;

		case CAN_TUBE_XRAY_ON:
            printUart(DBG_MSG_PC, "X-Ray On");
            break;

		case CAN_TUBE_XRAY_OFF:
            printUart(DBG_MSG_PC, "X-Ray Off");
            break;

        case CAN_TUBE_TANK_TEMP_READ:
        {
			char arrProtocol[UART_MSG_SIZE] = {0,};
			
			uint8_t temp = (uint8_t)canMsg.Value;
		
            printUart(DBG_MSG_PC, "Tank Temperature : %d", temp);

            //jehun - 20200717, printuart�� ����
            printUart(DBG_MSG_PC, "[SP_TEMP___%03d]", temp);
//			sprintf(arrProtocol, "[SP_TEMP___%04d]", temp);
			
            break;
        }
		case CAN_TUBE_COM_CHECK:
			printUart(DBG_MSG_PC, "Tube Comm Check Completed");
			break;
		case CAN_TUBE_SOUND_OFF:
			printUart(DBG_MSG_PC, "Tube Sound Off");
			break;
		case CAN_TUBE_SOUND_ON:
			printUart(DBG_MSG_PC, "Tube Sound On");
			break;
		case CAN_TUBE_KV_ADC_VALUE_CHECK:
			printUart(DBG_MSG_PC, "%d - kV ADC VALUE _ %d", canMsg.Debug, canMsg.Value);
			break;
		case CAN_TUBE_MA_ADC_VALUE_CHECK:
			printUart(DBG_MSG_PC, "%d - mA ADC VALUE _ %d", canMsg.Debug, canMsg.Value);
			break;
		case CAN_TUBE_RESET_ERROR_TABLE:
			printUart(DBG_MSG_PC, "ERROR CHECK TABLE RESET");
			break;
		case CAN_TUBE_RESET_INIRIAL_VALUE:
			printUart(DBG_MSG_PC, "TANK INITIAL COMPLETE");
			break;
		case CAN_TUBE_READY_TIME_CHECK:
			printUart(DBG_MSG_PC, "TANK READY TIME : %d", canMsg.Value);
			break;
		case CAN_TUBE_EXP_TIME_CHECK:
			printUart(DBG_MSG_PC, "TANK EXPOSURE TIME : %d", canMsg.Value);
			break;
		case CAN_TUBE_kV_MIN_VALUE_CHECK:
			printUart(DBG_MSG_PC, "TANK kV FEEDBACK MIN : %d", canMsg.Value);
			break;
		case CAN_TUBE_kV_MAX_VALUE_CHECK:
			printUart(DBG_MSG_PC, "TANK kV FEEDBACK MAX : %d", canMsg.Value);
			break;
		case CAN_TUBE_mA_MIN_VALUE_CHECK:
			printUart(DBG_MSG_PC, "TANK mA FEEDBACK MIN : %d", canMsg.Value);
			break;
		case CAN_TUBE_mA_MAX_VALUE_CHECK:
			printUart(DBG_MSG_PC, "TANK mA FEEDBACK MAX : %d", canMsg.Value);
			break;
		case CAN_TUBE_KV_STANDBY_ADC_VALUE_CHECK:
			printUart(DBG_MSG_PC, "%d - kV STANDBY ADC VALUE _ %d", canMsg.Debug, canMsg.Value);
			break;
		case CAN_TUBE_MA_STANDBY_ADC_VALUE_CHECK:
			printUart(DBG_MSG_PC, "%d - mA STANDBY ADC VALUE _ %d", canMsg.Debug, canMsg.Value);
			break;
		case CAN_TUBE_KV_MA_ERROR_DISABLE:
			if(canMsg.Value==1)
				printUart(DBG_MSG_PC, "KV_MA_ERROR ON");
			else
				printUart(DBG_MSG_PC, "KV_MA_ERROR OFF");
			break;
		case CAN_TUBE_TOTAL_ERROR_CHECK:
			if(canMsg.Value==1)
				printUart(DBG_MSG_PC, "TOTAL_ERROR ON");
			else
				printUart(DBG_MSG_PC, "TOTAL_ERROR OFF");
			break;
		case CAN_TUBE_FAN_ACTIVE_FLAG:
			if(canMsg.Value==1)
				printUart(DBG_MSG_PC, "FAN ACTIVE - ON");
			else
				printUart(DBG_MSG_PC, "FAN DEACTIVE - OFF");
			break;		
		case CAN_TUBE_FAN_ACTIVE_READ_FLAG:
			if(canMsg.Value==1)
				printUart(DBG_MSG_PC, "Current FAN ACTIVE - ON");
			else
				printUart(DBG_MSG_PC, "Current FAN DEACTIVE - OFF");
			break;
		case CAN_TUBE_FAN_ACTIVE_TEMP_SET:
            printUart(DBG_MSG_PC, "Set FAN ACTIVE Temperature : %d", (uint8_t)canMsg.Value);
            break;

        case CAN_TUBE_FAN_ACTIVE_TEMP_READ:
            printUart(DBG_MSG_PC, "FAN ACTIVE Temperature : %d", (uint8_t)canMsg.Value);
            break;
        default:
            printUart(DBG_MSG_PC, "################ GENERATOR CAN : UNEXPECTED RESULT !!");
            printUart(DBG_MSG_PC, "%x %x %x %x", \
                            canMsg.ExtId, canMsg.Cmd, canMsg.Value, canMsg.Debug);
            return FALSE;
            //break;
        }
    }
    else
    {	//200806 HWAN ADD
    	switch(canMsg.Cmd)
		{			
			case CAN_TUBE_KV_MA_ERROR_DISABLE:
				if(canMsg.Value==1)
					printUart(DBG_MSG_PC, "KV_MA_ERROR ON");
				else
					printUart(DBG_MSG_PC, "KV_MA_ERROR OFF");
				break;
			case CAN_TUBE_KV_ADD_VALUE_PERCENT:
				printUart(DBG_MSG_PC, "kV ADD VALUE PERCENT (+%d)", canMsg.Value);
				break;
			case CAN_TUBE_KV_ADD_VALUE_PERCENT_MINUS:
				printUart(DBG_MSG_PC, "kV ADD VALUE PERCENT (-%d)", canMsg.Value);
				break;
			case CAN_TUBE_MA_ADD_VALUE_PERCENT:
				printUart(DBG_MSG_PC, "MA ADD VALUE PERCENT (+%d)", canMsg.Value);
				break;
			case CAN_TUBE_MA_ADD_VALUE_PERCENT_MINUS:
				printUart(DBG_MSG_PC, "MA ADD VALUE PERCENT (-%d)", canMsg.Value);
				break;
			case CAN_TUBE_TOTAL_ERROR_CHECK:
				if(canMsg.Value==1)
					printUart(DBG_MSG_PC, "TOTAL_ERROR_ON");
				else
					printUart(DBG_MSG_PC, "TOTAL_ERROR_OFF");
				break;
			case CAN_TUBE_EEPROM_ONLY_ONE_CHK:
				printUart(DBG_MSG_PC, "EEPROM_ONLY_ONE_CHK (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_JUMP_STATUS:
				printUart(DBG_MSG_PC, "EEPROM_JUMP_STATUS (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_CHK_EXIST_APP:
				printUart(DBG_MSG_PC, "EEPROM_CHK_EXIST_APP (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_CONNECT_STATUS:
				printUart(DBG_MSG_PC, "EEPROM_CONNECT_STATUS (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_CHK_APP_SECTION:
				printUart(DBG_MSG_PC, "EEPROM_CHK_APP_SECTION (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_APP_FIRST_SECTION:
				printUart(DBG_MSG_PC, "EEPROM_APP_FIRST_SECTION (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_FIRSTBOOT_ADDRESS:
				printUart(DBG_MSG_PC, "EEPROM_FIRSTBOOT_ADDRESS (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_BUZZER_ADDRESS:
				printUart(DBG_MSG_PC, "EEPROM_BUZZER_ADDRESS (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_TEMPERATURE_ADDRESS:
				printUart(DBG_MSG_PC, "EEPROM_TEMPERATURE_ADDRESS (%d)", canMsg.Value);
				break;
			case CAN_TUBE_EEPROM_XRAY_CNT_ADDRESS:
				printUart(DBG_MSG_PC, "EEPROM_XRAY_CNT_ADDRESS (%d)", canMsg.Value);
				break;				
			default:
				printUart(DBG_MSG_PC, ERR_CODE_TUBE_RESP_ERROR" : %x %x %x %x", canMsg.ExtId, canMsg.Cmd, canMsg.Value, canMsg.Debug);		
        		return FALSE;
		} 
        
    }
	

    return TRUE;
}

/**
* @ Function Name : CAN_SendMessage_NoResponse
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool CAN_SendMessage_NoResponse(uint32_t extid, 
                        uint16_t cmd, uint32_t value, uint16_t debug)
{
    return CAN_ProcTxMessage(extid, cmd, value, debug);
}

/**
* @ Function Name : CAN_GeneratorErrorCheck
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CAN_GeneratorErrorCheck(unsigned short error)
{
    const tubeError_t *pCmd, *pChkCmd = NULL;
    int cnt = 0;
    
    printUart(DBG_MSG_PC, "[GEN] Received Response Error : ErrCode(0x%x)", error);

    for (pCmd = tubeErrorTable; pCmd < tubeErrorTable + TABLE_CNT; pCmd++, cnt++)
    {
        if (pCmd->cmd == error) 
		{
            if (!pChkCmd) 
			{
                pChkCmd = pCmd;
            }
            break;
        }
    }

    if (pChkCmd) 
	{
        printUart(DBG_MSG_PC, "%s", pChkCmd->strCode);

		if(pChkCmd->cmd < CAN_TUBE_KV_REF_OUT_HI_OVER_ERROR || pChkCmd->cmd > CAN_TUBE_mA_REF_OUT_LOW_OVER_ERROR)
		{
			gMachStat.bTubeErr = TRUE;
	        //sunghwan - V1.0.0e
	        //if(gMachStat.pTubeErrCode<pChkCmd) gMachStat.pTubeErrCode = pChkCmd;
	        gMachStat.pTubeErrCode = pChkCmd;
	        //sunghwan - V1.0.0e
		}        
    } 
	else 
	{
        printUart(DBG_MSG_PC, "[GEN] Unknown Protocol : 0x%04x", error);
    }
}

/**
* @ Function Name : CAN_ParseMessage
* @ Desc : 
* @ Param : 
* @ Return :
*/
/*__inline*/ void CAN_ParseMessage(void)
{
    static CAN_MsgTypedef msg;

    msg.ExtId = CAN_RxMessage.ExtId;
    msg.Cmd = (CAN_RxMessage.Data[0] << 8) + CAN_RxMessage.Data[1];
    msg.Value = (CAN_RxMessage.Data[2] << 24) + (CAN_RxMessage.Data[3] << 16) + \
                    (CAN_RxMessage.Data[4] << 8) + CAN_RxMessage.Data[5];
    msg.Debug = (CAN_RxMessage.Data[6] << 8) + CAN_RxMessage.Data[7];


    if (msg.ExtId == CAN_EXT_ID_SUBB_TO_SYSTEM)
    {
    	switch(msg.Debug)
		{
			case CAN_SUBB_RESPONSE:
			case COLLI_RESPONSE_RESULT:
			case COLLI_RESPONSE_ERROR:
			case COLLI_RESPONSE_PLUS:
			case COLLI_RESPONSE_MINUS:
				if (!CAN_Enqueue(&CollimCAN_Queue, msg))
	            {
	                printUart(DBG_MSG_PC, ERR_CODE_COL_QUEUE_OVERFLOW);
	                CAN_QueueInit(&CollimCAN_Queue);
	            }
	            break;
			default:
				printUart(DBG_MSG_PC, "[COL] CAN_ParseMessage Error : 0x%x Status : 0x%x", msg.Cmd , msg.Debug);
				break;
		}		
    }
    else if (msg.ExtId == CAN_EXT_ID_TUBE_TO_SYSTEM)
    {
        if (msg.Debug == CAN_TUBE_RESPONSE)
        {
            if (!CAN_Enqueue(&CAN_Queue, msg))
            {
                printUart(DBG_MSG_PC, ERR_CODE_TUBE_QUEUE_OVERFLOW);
                CAN_QueueInit(&CAN_Queue);
            }
            gMachStat.bTubeErr = FALSE;
            return;
        }//200324 HWAN
        else if(msg.Cmd >= CAN_TUBE_kV_HIGH_Output_WARNING && msg.Cmd <= CAN_TUBE_mA_Ref_Out_LOW_OVER_WARNING)
    	{
    		switch(msg.Cmd)
			{
				case CAN_TUBE_kV_HIGH_Output_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 102(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_kV_LOW_Output_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 103(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_kV_Ref_Out_HI_OVER_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 105(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_kV_Ref_Out_LOW_OVER_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 106(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_mA_HIGH_Output_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 202(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_mA_LOW_Output_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 203(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_mA_Ref_Out_HI_OVER_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 205(%d %%), %d count", msg.Debug,msg.Value);
					break;
				case CAN_TUBE_mA_Ref_Out_LOW_OVER_WARNING:
					printUart(DBG_MSG_PC, "GEN Warning 206(%d %%), %d count", msg.Debug,msg.Value);
					break;
			}
    	}
		else if(msg.Cmd==CAN_TUBE_KV_ADC_VALUE_CHECK||msg.Cmd==CAN_TUBE_MA_ADC_VALUE_CHECK||msg.Cmd==CAN_TUBE_KV_STANDBY_ADC_VALUE_CHECK||msg.Cmd==CAN_TUBE_MA_STANDBY_ADC_VALUE_CHECK)
		{
			if (!CAN_Enqueue(&CAN_Queue, msg))
            {
                printUart(DBG_MSG_PC, ERR_CODE_TUBE_QUEUE_OVERFLOW);
                CAN_QueueInit(&CAN_Queue);
            }
            gMachStat.bTubeErr = FALSE;
            return;
		}
        else
        {
            CAN_GeneratorErrorCheck(msg.Debug);
        }
    }
    else
    {
        printUart(DBG_MSG_PC, "CAN Extended Frame Error : Unknown ID(0x%08x)", msg.ExtId);
        return;
    }
}

/**
* @ Function Name : CAN_QueueInit
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CAN_QueueInit(CAN_QueueTypedef* queue)
{	
	queue->Head = 0;
	queue->Tail = 0;
	memset(queue->Message, NULL, sizeof(CAN_MsgTypedef) * CAN_QUEUE_SIZE);	
}

/**
* @ Function Name : CAN_QueueCnt
* @ Desc : 
* @ Param : 
* @ Return :
*/
static uint32_t CAN_QueueCnt(CAN_QueueTypedef* queue)
{
	return ((queue->Head - queue->Tail + CAN_QUEUE_SIZE) % CAN_QUEUE_SIZE);		
}

/**
* @ Function Name : CAN_Enqueue
* @ Desc : 
* @ Param : 
* @ Return :
*/
static bool CAN_Enqueue(CAN_QueueTypedef* queue, CAN_MsgTypedef Message)
{
	if (CAN_QueueCnt(queue) == CAN_QUEUE_SIZE - 1)
	{
		return FALSE;
	}	
	queue->Message[queue->Head++] = Message;
	queue->Head %= CAN_QUEUE_SIZE;
	/*if(sysInfo.bShowLog==TRUE && sysInfo.bShowQueueLog==TRUE)
	{
		printUart(DBG_MSG_PC, "queue->Head : %d, cmd : 0x%04x", queue->Head,Message.Cmd);
	}*/
	return TRUE;
}

/**
* @ Function Name : CAN_Dequeue
* @ Desc : 
* @ Param : 
* @ Return :
*/
static CAN_MsgTypedef CAN_Dequeue(CAN_QueueTypedef* queue)
{
	CAN_MsgTypedef message;
	
	message = queue->Message[queue->Tail++];
	queue->Tail %= CAN_QUEUE_SIZE;
	/*if(sysInfo.bShowLog==TRUE  && sysInfo.bShowQueueLog==TRUE)
	{
		printUart(DBG_MSG_PC, "queue->Tail : %d, cmd : 0x%04x", queue->Tail,message.Cmd);
	}*/
	return message;	
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

