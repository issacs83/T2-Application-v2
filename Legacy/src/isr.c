/*
*******************************************************************************************
* isr.c :
*******************************************************************************************
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
#include "motor.h"
#include "system.h"
#include "error_code.h"
#include "can.h"
#include "tube.h"
#include "serial.h"
#include "sensor.h"
#include "timer.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define MOTOR_R_ISR									TIM8_CC_IRQHandler
#define	MOTOR_R_GetCapture							TIM_GetCapture3
#define	MOTOR_R_SetCompare							TIM_SetCompare3

#define	MOTOR_V_ISR									TIM1_CC_IRQHandler
#define	MOTOR_V_GetCapture							TIM_GetCapture1
#define	MOTOR_V_SetCompare							TIM_SetCompare1

//20190618_myshin : SCB Rev 2.0
#if 1
#define	MOTOR_C_ISR									TIM2_IRQHandler
#define	MOTOR_C_GetCapture							TIM_GetCapture2
#define	MOTOR_C_SetCompare							TIM_SetCompare2
#else
#define	MOTOR_C_ISR									TIM4_IRQHandler
#define	MOTOR_C_GetCapture							TIM_GetCapture1
#define	MOTOR_C_SetCompare							TIM_SetCompare1
#endif

#define	MOTOR_S_ISR									TIM1_UP_TIM10_IRQHandler//TIM8_CC_IRQHandler
#define	MOTOR_S_GetCapture							TIM_GetCapture1
#define	MOTOR_S_SetCompare							TIM_SetCompare1

#define	MOTOR_T_ISR									TIM5_IRQHandler
#define	MOTOR_T_GetCapture							TIM_GetCapture1
#define	MOTOR_T_SetCompare							TIM_SetCompare1

#define	MOTOR_A_ISR									TIM1_BRK_TIM9_IRQHandler
#define	MOTOR_A_GetCapture							TIM_GetCapture1
#define	MOTOR_A_SetCompare							TIM_SetCompare1

#ifdef USE_MOTOR_CHINREST_VER
//jehun
//#define	MOTOR_CNS_ISR								TIM5_IRQHandler	
#define	MOTOR_CNS_ISR								TIM3_IRQHandler
#define	MOTOR_CNS_GetCapture						TIM_GetCapture3
#define	MOTOR_CNS_SetCompare						TIM_SetCompare3
#endif /* USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_HOR
#define	MOTOR_CWE_ISR								TIM1_TRG_COM_TIM11_IRQHandler
#define	MOTOR_CWE_GetCapture						TIM_GetCapture1
#define	MOTOR_CWE_SetCompare						TIM_SetCompare1
#endif /* USE_MOTOR_CHINREST_HOR */

#define	SENSOR_P_ISR								TIM8_BRK_TIM12_IRQHandler
#define	SENSOR_P_GetCapture							TIM_GetCapture1
#define	SENSOR_P_SetCompare							TIM_SetCompare1

#define	TUBE_ISR									TIM8_TRG_COM_TIM14_IRQHandler
#define	TUBE_GetCapture								TIM_GetCapture1
#define	TUBE_SetCompare								TIM_SetCompare1

#define	INT_TIMER_ISR								TIM6_DAC_IRQHandler

#define	ELAPSED_TIMER_ISR 							TIM7_IRQHandler

/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

///**
//  * @brief  This function handles NMI exception.
//  * @param  None
//  * @retval None
//  */
//void NMI_Handler(void)
//{
//}
//
///**
//  * @brief  This function handles Hard Fault exception.
//  * @param  None
//  * @retval None
//  */
//void HardFault_Handler(void)
//{
//  /* Go to infinite loop when Hard Fault exception occurs */
//  while (1)
//  {}
//}
//
///**
//  * @brief  This function handles Memory Manage exception.
//  * @param  None
//  * @retval None
//  */
//void MemManage_Handler(void)
//{
//  /* Go to infinite loop when Memory Manage exception occurs */
//  while (1)
//  {}
//}
//
///**
//  * @brief  This function handles Bus Fault exception.
//  * @param  None
//  * @retval None
//  */
//void BusFault_Handler(void)
//{
//  /* Go to infinite loop when Bus Fault exception occurs */
//  while (1)
//  {}
//}
//
///**
//  * @brief  This function handles Usage Fault exception.
//  * @param  None
//  * @retval None
//  */
//void UsageFault_Handler(void)
//{
//  /* Go to infinite loop when Usage Fault exception occurs */
//  while (1)
//  {}
//}
//
///**
//  * @brief  This function handles Debug Monitor exception.
//  * @param  None
//  * @retval None
//  */
//void DebugMon_Handler(void)
//{}
//
///**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @retval None
//  */
//void SVC_Handler(void)
//{}
//
///**
//  * @brief  This function handles PendSV_Handler exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{}


/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/******************************************************************************/
/*            STM32F2xx Peripherals Interrupt Handlers                        */
/******************************************************************************/

void MOTOR_T_ISR(void)
{
#ifdef USE_MOTOR_CHINREST_TS		
	if (TIM_GetITStatus(Motor_T.Timer.Periph, Motor_T.Timer.Source) != RESET) 
	{
		uint16_t nCh1Capt = 0;

		TIM_ClearITPendingBit(Motor_T.Timer.Periph, Motor_T.Timer.Source);
		
		nCh1Capt = MOTOR_T_GetCapture(Motor_T.Timer.Periph);
		
		if (Motor_T.Update) 
		{
			if ( (Motor_T.OrgSen.uStatus.bit.HistoryOn == 0) ||(Motor_T.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_T.MotorDir == DIR_LIMIT) 
				{
					Motor_T.nInitRunStep++;
				} 
				else 
				{
					Motor_T.nInitRunStep--;
				}
#if 1
				if ((Motor_T.nInitRunStep < -6400*2) ||(Motor_T.nInitRunStep > 6400*2) ) 
				{
					Motor_Stop(&Motor_T);
				}
#endif
			}
			if (Motor_T.Status == STATUS_RUN)
				Motor_GetNextRunCCR(&Motor_T);
			else
				Motor_GetNextOrgCCR(&Motor_T);		
		}

		if(Motor_T.NextCCR > 0 ){
			TIM_SetAutoreload(Motor_T.Timer.Periph, Motor_T.NextCCR + 0.5);
		}
		//jehun
		//MOTOR_T_SetCompare(Motor_T.Timer.Periph, nCh1Capt + Motor_T.NextCCR+ 0.5);
		Motor_T.Update = (bool)!Motor_T.Update;
		if(Motor_T.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOC, GPIO_Pin_6);
	}
#endif /* USE_MOTOR_CHINREST_TS */					  
}


void MOTOR_CNS_ISR(void)
{
#ifdef USE_MOTOR_CHINREST_VER
	if (TIM_GetITStatus(Motor_CNS.Timer.Periph, Motor_CNS.Timer.Source) != RESET) {
		uint16_t nCh3Capt = 0, temp1 = 0;

		TIM_ClearITPendingBit(Motor_CNS.Timer.Periph, Motor_CNS.Timer.Source);

		nCh3Capt = MOTOR_CNS_GetCapture(Motor_CNS.Timer.Periph);

		if (Motor_CNS.Update) {
			if ( (Motor_CNS.OrgSen.uStatus.bit.HistoryOn == 0) ||\
				(Motor_CNS.OrgSen.uStatus.bit.HistoryOff == 0) ) {
				if (Motor_CNS.MotorDir == DIR_LIMIT) {
					Motor_CNS.nInitRunStep++;
				} else {
					Motor_CNS.nInitRunStep--;
				}
#if 1 //20190701_myshin : motor change
				if ( (Motor_CNS.nInitRunStep < -88000*2) ||\
					(Motor_CNS.nInitRunStep > 88000*2) ) {
					Motor_Stop(&Motor_CNS);
				}
#endif
			}

			if (Motor_CNS.Status == STATUS_RUN) {
				Motor_GetNextRunCCR(&Motor_CNS);
			} else {
				Motor_GetNextOrgCCR(&Motor_CNS);
			}
		}

		//MOTOR_CNS_SetCompare(Motor_CNS.Timer.Periph, nCh3Capt + Motor_CNS.NextCCR + 0.5);
		if(Motor_CNS.NextCCR >= 0 ){
			TIM_SetAutoreload(Motor_CNS.Timer.Periph, Motor_CNS.NextCCR + 0.5);
		}
//		TIM_ARRPreloadConfig(Motor_CNS.Timer.Periph, Bit_SET);
		//GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
		Motor_CNS.Update = (bool)!Motor_CNS.Update;
//		MOTOR_CNS_SetCompare(Motor_CNS.Timer.Periph, Motor_CNS.NextCCR + 0.5);
		//temp1 =  Motor_CNS.NextCCR + 0.5;
		//TIM_SetAutoreload(Motor_CNS.Timer.Periph, temp1); 	// ARR
		if(Motor_CNS.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOB,  GPIO_Pin_0);
	}
#endif // USE_MOTOR_CHINREST_VER //
}


/******************************************************************************/
/*            UART                                                  */              
/******************************************************************************/
/**
  * @brief  This function handles Usart1 Handler.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    uint8_t data;

    if (USART_GetITStatus(UART_PC, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(UART_PC, USART_IT_RXNE);
        data = USART_ReceiveData(UART_PC);
        UART_ParseMessage(data);
    }
}

#ifdef USE_TABLET_PC
/**
  * @brief  This function handles Uart2 Handler.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
    uint8_t data;

    if(USART_GetITStatus(UART_TABLET, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(UART_TABLET, USART_IT_RXNE);
        data = USART_ReceiveData(UART_TABLET);
        UART_ParseMessage(data);
    }
}
#endif /* USE_TABLET_PC */


/******************************************************************************/
/*            CAN                                                    */              
/******************************************************************************/
/**
  * @brief  This function handles can2 Handler.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN, CAN_IT_FMP0);
		CAN_Receive(CAN, CAN_FIFO0, &CAN_RxMessage);

		if ((CAN_RxMessage.IDE == CAN_ID_EXT) && (CAN_RxMessage.DLC == CAN_FRAME_LENGTH_BYTE) && ((CAN_RxMessage.ExtId == CAN_EXT_ID_SUBB_TO_SYSTEM) || (CAN_RxMessage.ExtId == CAN_EXT_ID_TUBE_TO_SYSTEM)))
		{
			CAN_ParseMessage();
		}
		else
		{
			if(sysInfo.bShowLog==TRUE)
			{
	            printUart(DBG_MSG_PC, "CAN Rx Error : CAN_RxMessage.DLC(%d), ExtID(0x%08x)", \
	            			CAN_RxMessage.DLC, CAN_RxMessage.ExtId);
			}
            UART_SendMessage(DBG_MSG_PC, ERR_CODE_CAN_QUEUE_ERROR);
		}
	}

}

/******************************************************************************/
/*            Motor                                                  */              
/******************************************************************************/
#ifdef USE_MOTOR_CHINREST_HOR
/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void MOTOR_CWE_ISR(void)
{	
	if (TIM_GetITStatus(Motor_CWE.Timer.Periph, Motor_CWE.Timer.Source) != RESET) {
		uint16_t nCh4Capt = 0, temp1 = 0;
		
		TIM_ClearITPendingBit(Motor_CWE.Timer.Periph, Motor_CWE.Timer.Source);
		
		nCh4Capt = MOTOR_CWE_GetCapture(Motor_CWE.Timer.Periph);
		
		if (Motor_CWE.Update) {
			if ( (Motor_CWE.OrgSen.uStatus.bit.HistoryOn == 0) || (Motor_CWE.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_CWE.MotorDir == DIR_LIMIT) 
				{
					Motor_CWE.nInitRunStep++;
				} 
				else 
				{
					Motor_CWE.nInitRunStep--;
				}

				if ( (Motor_CWE.nInitRunStep < -25600*2) ||(Motor_CWE.nInitRunStep > 25600*2) ) 
				{
					Motor_Stop(&Motor_CWE);
				}
			}
			if (Motor_CWE.Status == STATUS_RUN) 
			{
				Motor_GetNextRunCCR(&Motor_CWE);
			} 
			else 
			{
				Motor_GetNextOrgCCR(&Motor_CWE);
			}
		}
		
		Motor_CWE.Update = (bool)!Motor_CWE.Update;
//		MOTOR_CWE_SetCompare(Motor_CWE.Timer.Periph, Motor_CWE.NextCCR + 0.5);
		temp1 =  Motor_CWE.NextCCR + 0.5;
		TIM_SetAutoreload(Motor_CWE.Timer.Periph, temp1); 	// ARR
		if(Motor_CWE.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOB,  GPIO_Pin_9);
	}
}
#endif /* USE_MOTOR_CHINREST_HOR */

/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void MOTOR_R_ISR(void)
{
	if (TIM_GetITStatus(Motor_R.Timer.Periph, Motor_R.Timer.Source) != RESET) 
	{
		uint16_t nCh3Capt = 0, temp1 = 0;

		TIM_ClearITPendingBit(Motor_R.Timer.Periph, Motor_R.Timer.Source);
		nCh3Capt = MOTOR_R_GetCapture(Motor_R.Timer.Periph);

		if (Motor_R.Update) 
		{

			if ( (Motor_R.OrgSen.uStatus.bit.HistoryOn == 0) ||(Motor_R.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_R.MotorDir == DIR_LIMIT) 
				{
					Motor_R.nInitRunStep++;
				} 
				else 
				{
					Motor_R.nInitRunStep--;
				}

				if ((Motor_R.nInitRunStep < -72000*2) ||(Motor_R.nInitRunStep > 72000*2) ) 
				{
					Motor_Stop(&Motor_R);
				}
			}
			if (Motor_R.Status == STATUS_ARCH) 
			{
				Motor_GetNextArchCCR(&Motor_R);
			} 
			else if (Motor_R.Status == STATUS_RUN) 
			{
				//Motor_GetNextRunCCR(&Motor_R);
				Motor_GetNextRunCCR_R(&Motor_R);
			} 
			else if (Motor_R.Status == STATUS_CT) 
			{
				Motor_Get_NextCT_CCR(&Motor_R);
			} 
			else 
			{
				//Motor_GetNextOrgCCR(&Motor_R);
				if(CtParam.bReverse_Status==SET && CurCaptureMode==CAPTURE_CT)
					Motor_GetNextReverse_OrgCCR_R(&Motor_R);
				else
					Motor_GetNextOrgCCR_R(&Motor_R);
			}
		}

		//MOTOR_R_SetCompare(Motor_R.Timer.Periph, nCh3Capt + Motor_R.NextCCR + 0.5);
		Motor_R.Update = (bool)!Motor_R.Update;
//  		MOTOR_R_SetCompare(Motor_R.Timer.Periph, Motor_R.NextCCR + 0.5);
		temp1 =  Motor_R.NextCCR + 0.5;
		TIM_SetAutoreload(Motor_R.Timer.Periph, temp1); 	// ARR
		if(Motor_R.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOC,  GPIO_Pin_8);
	}
}

/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void MOTOR_V_ISR(void)
{
    if (TIM_GetITStatus(Motor_V.Timer.Periph, Motor_V.Timer.Source) != RESET) 
	{
		uint16_t nCh1Capt = 0, temp1 = 0;

        TIM_ClearITPendingBit(Motor_V.Timer.Periph, Motor_V.Timer.Source);

        if ((Motor_V.Timer.Periph == TIM2) || (Motor_V.Timer.Periph == TIM5))
            TIM_SetCounter(Motor_V.Timer.Periph, 0);
        else
            nCh1Capt = MOTOR_V_GetCapture(Motor_V.Timer.Periph);

        if (Motor_V.Update) {
			if ( (Motor_V.OrgSen.uStatus.bit.HistoryOn == 0) ||(Motor_V.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_V.MotorDir == DIR_LIMIT) 
				{
					Motor_V.nInitRunStep++;
				} 
				else 
				{
					Motor_V.nInitRunStep--;
				}

				if ( (Motor_V.nInitRunStep < -52000*2) ||(Motor_V.nInitRunStep > 52000*2) ) 
				{
					Motor_Stop(&Motor_V);
				}
			}
            if (Motor_V.Status == STATUS_ARCH) 
			{
                Motor_V.RunStep++;

                if (Motor_V.MotorDir == DIR_LIMIT)
                    Motor_V.CurrentStep++;
                else
                    Motor_V.CurrentStep--;
        
                if (Motor_V.RunStep == Motor_V.ArchParam.IndexStep) 
				{
                    Motor_V.RunStep = 0;
                    if (Motor_V.ArchParam.ArrayIndex > Motor_R.ArchParam.ArrayIndex)
                        return;
                    
                    Motor_V.ArchParam.ArrayIndex++;

                    if (Motor_V.ArchParam.ArrayIndex == Motor_V.ArchParam.TotalSize) 
					{
                        Motor_Stop(&Motor_V);
                        return;
                    } 
					else 
					{
                        Motor_SetArchParam(&Motor_V);
                        Motor_V.NextCCR = Motor_V.ArchParam.IndexCcr;
                    }
                }       
            } 
			else if (Motor_V.Status == STATUS_RUN) 
			{
                Motor_GetNextRunCCR(&Motor_V);
            } 
			else 
			{
                Motor_GetNextOrgCCR(&Motor_V);
            }
        }
//		MOTOR_V_SetCompare(Motor_V.Timer.Periph, Motor_V.NextCCR + 0.5);
		temp1 =  Motor_V.NextCCR + 0.5;
		TIM_SetAutoreload(Motor_V.Timer.Periph, temp1); 	// ARR
		
		//MOTOR_V_SetCompare(Motor_V.Timer.Periph, nCh1Capt + Motor_V.NextCCR + 0.5);
		Motor_V.Update = (bool)!Motor_V.Update;
		if(Motor_V.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOA, GPIO_Pin_8);
    }
}

/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void MOTOR_C_ISR(void)  // Timer problem  (schematic opposite )
{
	uint16_t capture = 0, temp1 = 0;
	
	if (TIM_GetITStatus(Motor_C.Timer.Periph, Motor_C.Timer.Source) != RESET)
	{
		TIM_ClearITPendingBit(Motor_C.Timer.Periph, Motor_C.Timer.Source);

// jehun 20200612
//		if((Motor_C.Timer.Periph == TIM2) || (Motor_C.Timer.Periph == TIM5))
//			TIM_SetCounter(Motor_C.Timer.Periph, 0);
//		else
			capture = MOTOR_C_GetCapture(Motor_C.Timer.Periph);		

		if (Motor_C.Update)
		{
			if ( (Motor_C.OrgSen.uStatus.bit.HistoryOn == 0) ||(Motor_C.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_C.MotorDir == DIR_LIMIT) 
				{
					Motor_C.nInitRunStep++;
				} 
				else 
				{
					Motor_C.nInitRunStep--;
				}

				if ((Motor_C.nInitRunStep < -28937*2) ||(Motor_C.nInitRunStep > 28937*2)) 
				{
					Motor_Stop(&Motor_C);
				}
			}
			if(Motor_C.Status == STATUS_RUN)
				Motor_GetNextRunCCR(&Motor_C);
			else
				Motor_GetNextOrgCCR(&Motor_C);		
		}

//		MOTOR_C_SetCompare(Motor_C.Timer.Periph, Motor_C.NextCCR + 0.5);
//		temp1 =  Motor_C.NextCCR + 0.5;
		//jehun -20200722
		TIM_SetAutoreload(Motor_C.Timer.Periph, Motor_C.NextCCR + 0.5); 	// ARR
		
//		MOTOR_C_SetCompare(Motor_C.Timer.Periph, capture + Motor_C.NextCCR + 0.5);
		Motor_C.Update = (bool)!Motor_C.Update;
		if(Motor_C.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOA, GPIO_Pin_1);
	}
}

/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void MOTOR_S_ISR(void)
{
	if (TIM_GetITStatus(Motor_S.Timer.Periph, Motor_S.Timer.Source) != RESET)
	{
		uint16_t nCh1Capt = 0, temp1 = 0;
		
		TIM_ClearITPendingBit(Motor_S.Timer.Periph, Motor_S.Timer.Source);
		
		nCh1Capt = MOTOR_S_GetCapture(Motor_S.Timer.Periph);
		
		if(Motor_S.Update)
		{
			if ( (Motor_S.OrgSen.uStatus.bit.HistoryOn == 0) ||(Motor_S.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_S.MotorDir == DIR_LIMIT) 
				{
					Motor_S.nInitRunStep++;
				} 
				else 
				{
					Motor_S.nInitRunStep--;
				}

				if ((Motor_S.nInitRunStep < -28937*2) ||(Motor_S.nInitRunStep > 28937*2) )
				{
					Motor_Stop(&Motor_S);
				}
			}
			if(Motor_S.Status == STATUS_RUN)
				Motor_GetNextRunCCR(&Motor_S);
			else
				Motor_GetNextOrgCCR(&Motor_S);		
		}

//		MOTOR_S_SetCompare(Motor_S.Timer.Periph, Motor_S.NextCCR + 0.5);
//		temp1 =  Motor_S.NextCCR + 0.5;

		//jehun - 20200722
		TIM_SetAutoreload(Motor_S.Timer.Periph, Motor_S.NextCCR + 0.5); 	// ARR
//		MOTOR_S_SetCompare(Motor_S.Timer.Periph, nCh1Capt + Motor_S.NextCCR + 0.5);

		Motor_S.Update = (bool)!Motor_S.Update;
		if(Motor_S.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOB, GPIO_Pin_8);
	}
}

#ifdef USE_MOTOR_GANTRY_MS
/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void MOTOR_A_ISR(void)
{
	uint16_t capture = 0, temp1 = 0;
	
	if (TIM_GetITStatus(Motor_A.Timer.Periph, Motor_A.Timer.Source) != RESET)
	{
		TIM_ClearITPendingBit(Motor_A.Timer.Periph, Motor_A.Timer.Source );
		
		if((Motor_A.Timer.Periph == TIM2) || (Motor_A.Timer.Periph == TIM5))
			TIM_SetCounter(Motor_A.Timer.Periph, 0);
		else
			capture = MOTOR_A_GetCapture(Motor_A.Timer.Periph);
		
		if(Motor_A.Update)
		{
			if ( (Motor_A.OrgSen.uStatus.bit.HistoryOn == 0) ||(Motor_A.OrgSen.uStatus.bit.HistoryOff == 0) ) 
			{
				if (Motor_A.MotorDir == DIR_LIMIT) 
				{
					Motor_A.nInitRunStep++;
				} 
				else 
				{
					Motor_A.nInitRunStep--;
				}

				if ((Motor_A.nInitRunStep < -27200*2) ||(Motor_A.nInitRunStep > 27200*2) ) 
				{
					Motor_Stop(&Motor_A);
				}
			}
			if(Motor_A.Status == STATUS_RUN)
				Motor_GetNextRunCCR(&Motor_A);
			else
				Motor_GetNextOrgCCR(&Motor_A);
		}

//		MOTOR_A_SetCompare(Motor_A.Timer.Periph, Motor_A.NextCCR + 0.5);
		temp1 =  Motor_A.NextCCR + 0.5;
		TIM_SetAutoreload(Motor_A.Timer.Periph, temp1); 	// ARR
		
		//MOTOR_A_SetCompare(Motor_A.Timer.Periph, capture + Motor_A.NextCCR + 0.5);
		Motor_A.Update = (bool)!Motor_A.Update;
		if(Motor_A.Status != STATUS_DELAY) GPIO_ToggleBits(GPIOE, GPIO_Pin_5);
	}
}
#endif /* USE_MOTOR_GANTRY_MS */


/******************************************************************************/
/*            CT Sensor                                          */              
/******************************************************************************/
/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void SENSOR_P_ISR(void)
{
#if defined(USE_CT_XRAY_PULSED_MODE)
    uint16_t capture = 0;

	if (TIM_GetITStatus(Sensor_C.Timer.Periph, SENSOR_P_SOURCE) != RESET)
	{
		TIM_ClearITPendingBit(Sensor_C.Timer.Periph, SENSOR_P_SOURCE);
		
		if ((Sensor_C.Timer.Periph == TIM2) || (Sensor_C.Timer.Periph == TIM5))
		{
			TIM_SetCounter(Sensor_C.Timer.Periph, 0);
		}
        else
        {
			capture = SENSOR_P_GetCapture(Sensor_C.Timer.Periph);
        }
        
		if (!Sensor_C.Update)
		{
			SENSOR_P_SetCompare(Sensor_C.Timer.Periph, capture + Sensor_C.Param.SyncHighTimeCcr);
		}
		else
		{
			SENSOR_P_SetCompare(Sensor_C.Timer.Periph, capture + Sensor_C.Param.SyncLowTimeCcr);
		}		
	}
	Sensor_C.Update = !Sensor_C.Update;        
#else /* not USE_CT_XRAY_PULSED_MODE */
    uint16_t capture = 0;

    if (TIM_GetITStatus(Sensor_P.Timer.Periph, SENSOR_P_SOURCE) != RESET) 
    {
        TIM_ClearITPendingBit(Sensor_P.Timer.Periph, SENSOR_P_SOURCE);

        if ((Sensor_P.Timer.Periph == TIM2) || (Sensor_P.Timer.Periph == TIM5)) 
        {      
            TIM_SetCounter(Sensor_P.Timer.Periph, 0);
        } 
        else 
        {
            capture = SENSOR_P_GetCapture(Sensor_P.Timer.Periph);
        }
    }

    if (!Sensor_P.Update) 
    {
        SENSOR_P_SetCompare(Sensor_P.Timer.Periph, capture + Sensor_P.Trigger.SyncHighTimeCcr);
    }
    else
    {
        Sensor_P.Index++;
        Sensor_P.Trigger.VariableLowTimeCcr = Sensor_P.CaptureCcr[Motor_R.ArchParam.ArrayIndex];
        
        SENSOR_P_SetCompare(Sensor_P.Timer.Periph, capture + Sensor_P.Trigger.VariableLowTimeCcr + 0.5);
		Sensor_P.Count++;
    }
    Sensor_P.Update = (bool)!Sensor_P.Update;
#endif /* USE_CT_XRAY_PULSED_MODE */	
}


#ifndef USE_TUBE_PPS_TYPE_IO
/******************************************************************************/
/*            Tube                                                 */              
/******************************************************************************/
/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void TUBE_ISR(void)
{
	uint16_t capture = 0;
	
	if (TIM_GetITStatus(Tube.Timer.Periph, TUBE_SOURCE) != RESET)
	{
		TIM_ClearITPendingBit(Tube.Timer.Periph, TUBE_SOURCE );

		if((Tube.Timer.Periph == TIM2) || (Tube.Timer.Periph == TIM5))
			TIM_SetCounter(Tube.Timer.Periph, 0);
		else
			capture = TUBE_GetCapture(Tube.Timer.Periph);
		
		if(Tube.Update) 
		{
			TUBE_SetCompare(Tube.Timer.Periph, capture + Tube.Param.XonTimeCcr);
		}
		else
		{
			TUBE_SetCompare(Tube.Timer.Periph, capture + Tube.Param.XoffTimeCcr);
		}
	}

	Tube.Update = (bool)!Tube.Update;
}
#endif /* not USE_TUBE_PPS_TYPE_IO */


/******************************************************************************/
/*            Timer                                                 */              
/******************************************************************************/
/**
  * @brief  This function handles timer Handler.
  * @param  None
  * @retval None
  */
void INT_TIMER_ISR(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        IntTimer.Count++;	
    }	
}

void ELAPSED_TIMER_ISR(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) 
	{
		typElapsedTimer.endMs++;
		if (typElapsedTimer.endMs >= kMax_uint32_t) 
		{
			typElapsedTimer.endMs = 0;
		}
		
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update );
	}
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

