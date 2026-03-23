/*
*******************************************************************************************
* timer.h :
*******************************************************************************************
* Copyright (C) 2016-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2018-11-12
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __TIMER_H__
#define __TIMER_H__


/* Exported define --------------------------------------------------- */
#define	TIM_COUNT_CLOCK				1250000
#define	TIM_TOGGLE					2
#define	sec_To_msec					1000
#define	msec_To_sec					1 / 1000
#define	Hz_To_CCR(Hz)				(double) TIM_COUNT_CLOCK / (TIM_TOGGLE * Hz)
#define	ms_To_CCR(ms)				(double) (TIM_COUNT_CLOCK * ms) / sec_To_msec
//#define	ms_To_CCR(ms)			(double) TIM_COUNT_CLOCK *  ((double) ms / sec_To_msec)
#define LowSig_Hz_To_CCR(Hz) 		(double)TIM_COUNT_CLOCK / Hz

#define kMax_uint32_t  0xFFFFFFFF


/* Exported types ----------------------------------------------------*/
typedef struct
{
	Timer_Typedef		Timer;
	uint32_t			Period;
	uint32_t			Delay;
	uint32_t			Count;
    bool               bLampTimer;
    bool               bLampEnable;
} IntTimer_Typedef;

typedef struct
{
	Timer_Typedef Timer;
	uint32_t Period;
	uint32_t startMs;
	uint32_t endMs;
#ifdef BUILD_TYPE_DEBUG
    uint32_t nExposeTime;
#endif /* BUILD_TYPE_DEBUG */
} ElapsedTimer_Typedef;


/* Exported macro ---------------------------------------------------*/
/* Exported variables -------------------------------------------------*/
extern IntTimer_Typedef IntTimer;
extern ElapsedTimer_Typedef typElapsedTimer;


/* Exported functions ------------------------------------------------ */
void IntTimer_Config(void);
void IntTimer_Delay(uint32_t delay);
bool IntTimer_GetStatus(void);
void IntTimer_Stop(void);
void LampTimer_Enable(void);
void LampErrorTimer_Enable(int8_t Error_Case , int8_t Timer_D);
bool LampTimer_GetStatus(void);
#ifdef USE_TUBE_PPS_TYPE_IO
void IntTimer_Disable(void);
#endif /* USE_TUBE_PPS_TYPE_IO */            
void delay_us(unsigned long us);
void delay_ms(unsigned long ms);
char Systick_Configuration(void);
void SysTick_DelayMs(unsigned int ms);
void SysTick_DelayUs(unsigned int us);

void vElapsedTimerConfig(void);
uint32_t vSetCurrentMilliSec(void);
uint32_t nElapsedMilliSec(uint32_t ulStartTime);
void ElapseTimerOnOff(bool bEnable);
#ifdef USE_AGING_MODE
void SysSetTime(_Bool bEnable);
void SysTime_MesureStart(const char *pFuncName, int nLine);
void SysTime_MesureEnd(const char *pFuncName, int nLine);
#endif /* USE_AGING_MODE */


#endif /* __TIMER_H__ */
