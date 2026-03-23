/*
*******************************************************************************
* cmd_geoalign.c : Geometry alignment dispatcher
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SetGeoAlignParam -- top-level dispatcher that delegates
*             axis-specific commands to sub-handlers in cmd_geoalign_axes.c.
*
* @ Revision History :
*       1) Extracted from cmd_geoalign_cal.c for modular build --- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
#include "cmd_geoalign_cal_priv.h"

/* ---------------------------------------------------------------------------
 * Shared module-level variables (definitions)
 * -------------------------------------------------------------------------*/
#ifdef CEPH_SCAN_LOOPBACK
bool g_bCephLoopBack;
double g_dDetectorStartPos;
double g_dDetecotrRatio;
double g_dCollimatorStartPos;
double g_dCollimatorRatio;
uint32_t g_nCollimatorDistance;
#endif /* CEPH_SCAN_LOOPBACK */

bool bClose = RESET, bOpen = RESET;
bool b2ndColStart = RESET;

/* ---------------------------------------------------------------------------
 * UART_SetGeoAlignParam -- geometry alignment command dispatcher
 *
 * Processes queued commands during geometry alignment mode.
 * Delegates axis-specific handling to sub-functions to keep this file
 * focused on dispatching logic.
 * -------------------------------------------------------------------------*/
bool UART_SetGeoAlignParam(void)
{
	UART_MsgTypedef message;
	bool bRet = RESET, bChin_res = RESET;;

	while(MSG_QueueCnt(&UART_Queue))
    {
        message = MSG_Dequeue(&UART_Queue);
		if (strstr(message.Data, UART_MODE_EXIT))
		{
    		CurCaptureMode = CAPTURE_PANO;
			bRet = SET;
		}
		/* --- R-axis commands --- */
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_ROT00) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_ROT90) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_STEP_STOP) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_DEC_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_RAXIS_INC_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_GET_RAXIS_OFFSET))
        {
            if (!GeoAlign_HandleRAxis(&message))
                return RESET;
        }
		/* --- V-axis + canine init commands --- */
        else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_VAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_VAXIS_DEC_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_VAXIS_INC_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_GET_VAXIS_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_VAXIS_STEP_STOP) ||
                 strstr(message.Data, UART_GEO_ALIGN_VAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_VAXIS_RESET) ||
                 strstr(message.Data, UART_CANINE_ALIGN_MOVE_VAXIS_INIT) ||
                 strstr(message.Data, UART_CANINE_ALIGN_VAXIS_STEP_UP) ||
                 strstr(message.Data, UART_CANINE_ALIGN_VAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_CANINE_ALIGN_VAXIS_STOP) ||
                 strstr(message.Data, UART_CANINE_ALIGN_VAXIS_SET) ||
                 strstr(message.Data, UART_CANINE_ALIGN_VAXIS_RESET))
        {
            if (!GeoAlign_HandleVAxis(&message))
                return RESET;
        }
		/* --- Ceph R-axis alignment --- */
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_UP) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_DOWN) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_STOP) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_SET) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_READ) ||
                 strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_INIT) ||
                 strstr(message.Data, UART_GEO_CEPH_POSITION_MOVE))
        {
            if (!GeoAlign_HandleCephRAxis(&message))
                return RESET;
        }
		/* --- CT axes (P-axis, R-axis, fast R-axis, patient axes) --- */
        else if (strstr(message.Data, UART_CT_ALIGN_MOVE_CT_POS) ||
                 strstr(message.Data, UART_CT_ALIGN_PAXIS_SET_DOWN) ||
                 strstr(message.Data, UART_CT_ALIGN_PAXIS_SET_UP) ||
                 strstr(message.Data, UART_CT_ALIGN_PAXIS_RESET) ||
                 strstr(message.Data, UART_CT_ALIGN_PAXIS_GET_OFFSET) ||
                 strstr(message.Data, UART_CT_ALIGN_RAXIS_SET_DOWN) ||
                 strstr(message.Data, UART_CT_ALIGN_RAXIS_SET_UP) ||
                 strstr(message.Data, UART_CT_ALIGN_RAXIS_RESET) ||
                 strstr(message.Data, UART_CT_ALIGN_RAXIS_GET_OFFSET) ||
                 strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_SET_DOWN) ||
                 strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_SET_UP) ||
                 strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_RESET) ||
                 strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_GET_OFFSET) ||
                 strstr(message.Data, UART_CT_PATIENT_PAXIS_SET_DOWN) ||
                 strstr(message.Data, UART_CT_PATIENT_PAXIS_SET_UP) ||
                 strstr(message.Data, UART_CT_PATIENT_PAXIS_RESET) ||
                 strstr(message.Data, UART_CT_PATIENT_PAXIS_GET_OFFSET) ||
                 strstr(message.Data, UART_CT_PATIENT_RAXIS_SET_DOWN) ||
                 strstr(message.Data, UART_CT_PATIENT_RAXIS_SET_UP) ||
                 strstr(message.Data, UART_CT_PATIENT_RAXIS_RESET) ||
                 strstr(message.Data, UART_CT_PATIENT_RAXIS_GET_OFFSET))
        {
            if (!GeoAlign_HandleCTAxes(&message))
                return RESET;
        }
		/* --- Ceph around shot, tube params, 2nd coli positions --- */
        else if (strstr(message.Data, UART_CEPH_RAXIS_ALIGN_AROUND_SHOT) ||
                 strstr(message.Data, UART_TUBE_VOLTAGE) ||
                 strstr(message.Data, UART_TUBE_CURRENT) ||
                 strstr(message.Data, UART_COLLIMATOR_OPEN) ||
                 strstr(message.Data, UART_CEPH_2COL_START_POS) ||
                 strstr(message.Data, UART_CEPH_2COL_END_POS) ||
                 strstr(message.Data, UART_CEPH_DET_START_POS) ||
                 strstr(message.Data, UART_CEPH_DET_END_POS))
        {
            if (!GeoAlign_HandleCommon(&message, &bChin_res))
                return RESET;
        }
		/* --- CNS/CWE axes (Pano + CT chinrest offsets) --- */
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_DEC) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_INC) ||
                 strstr(message.Data, UART_GEO_ALIGN_GET_PANO_CNSAXIS_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_DEC) ||
                 strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_INC) ||
                 strstr(message.Data, UART_GEO_ALIGN_GET_PANO_CWEAXIS_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_DEC) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_INC) ||
                 strstr(message.Data, UART_GEO_ALIGN_GET_CT_CNSAXIS_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_DEC) ||
                 strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_INC) ||
                 strstr(message.Data, UART_GEO_ALIGN_GET_CT_CWEAXIS_OFFSET))
        {
            if (!GeoAlign_HandleCNSCWEAxes(&message, &bChin_res))
                return RESET;
        }
		/* --- Laser beams --- */
        else if (strstr(message.Data, UART_LASER_ON))
        {
			LaserControl(TYPE_HEAD_PANO, SET);
			LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_LASER_OFF))
        {
			LaserControl(TYPE_HEAD_PANO, RESET);
			LaserControl(TYPE_FOOT, RESET);
        }
		else if (strstr(message.Data, UART_HEAD_BEAM_CUSTOM_ON))
        {
        	if (strchr(&(message.Data[13]), 'P'))
			{
				LaserControl(TYPE_HEAD_PANO, SET);
			}
			else if(strchr(&(message.Data[13]), 'C'))
			{
				LaserControl(TYPE_HEAD_CT, SET);
			}
			else if(strchr(&(message.Data[13]), 'S'))
			{
				LaserControl(TYPE_HEAD_CEPH, SET);
			}
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_CUSTOM_OFF))
        {
        	if (strchr(&(message.Data[13]), 'P'))
			{
				LaserControl(TYPE_HEAD_PANO, RESET);
			}
			else if(strchr(&(message.Data[13]), 'C'))
			{
				LaserControl(TYPE_HEAD_CT, RESET);
			}
			else if(strchr(&(message.Data[13]), 'S'))
			{
				LaserControl(TYPE_HEAD_CEPH, RESET);
			}
        }
		/* --- Tube temp --- */
		else if (strstr(message.Data, UART_GET_TUBE_TEMP))
		{
			CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
		}
		/* --- Ear rod --- */
		else if (strstr(message.Data, UART_EAR_ROD_PUSH))
		{
			EarRod_Control(EAR_ROD_PUSH);
			IntTimer_Delay(300);
			while(!IntTimer_GetStatus());
		}
		else if (strstr(message.Data, UART_EAR_ROD_CHILD_PUSH))
		{
			EarRod_Control(EAR_ROD_CHILD);
			IntTimer_Delay(300);
			while(!IntTimer_GetStatus());
		}
		else if (strstr(message.Data, UART_EAR_ROD_RELEASE))
		{
			EarRod_Control(EAR_ROD_RELEASE);
			IntTimer_Delay(300);
			while(!IntTimer_GetStatus());
		}
		/* --- T-axis (temple support) --- */
		else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_STEP_UP) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_STEP_DOWN) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_STEP_STOP) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_MOVE_STEP) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_OFFSET) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_CHILD_SET) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_CHILD_RESET) ||
                 strstr(message.Data, UART_GEO_ALIGN_TAXIS_CHILD_OFFSET))
        {
            if (!GeoAlign_HandleTAxis(&message))
                return RESET;
        }
		/* --- Chinrest move --- */
#ifdef USE_MOTOR_CHINREST_HOR
		else if (strstr(message.Data, UART_CMD_CHINREST_HOR_MOVE))
		{
			int32_t nDistance = atoi(&(message.Data[12]));

			printUart(DBG_MSG_PC, "H-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				if (strchr(&(message.Data[11]), 'R'))
					nDistance = ~nDistance + 1;

				Motor_ControlChinrest(RESET, MOTOR_CWE_INIT_POSITION + nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_HOR */
#ifdef USE_MOTOR_CHINREST_VER
		else if (strstr(message.Data, UART_CMD_CHINREST_VER_MOVE))
		{
			int32_t nDistance = atoi(&(message.Data[12]));

			printUart(DBG_MSG_PC, "V-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				if (strchr(&(message.Data[11]), 'D'))
					nDistance = ~nDistance + 1;

				Motor_ControlChinrest(SET, MOTOR_CNS_CT_INIT_POSITION + nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_VER */
        else
        {
            printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            return RESET;
        }

        message.Data[1] = 'E';
        printUart(DBG_MSG_PC, message.Data);
    }

	return bRet;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
