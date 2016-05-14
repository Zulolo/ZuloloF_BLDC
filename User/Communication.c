/******************************************************************************
 * @file     Communication.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/12/10 21:13p $ 
 * @brief    SPI Communication for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2014 Zulolo Technology Corp. All rights reserved.
*****************************************************************************/   
#define __USED_BY_COMMUNICATION_C__
#include "Communication.h"
#include <string.h>

/*
 * The communication works like this:
 * If one unit transfer is finished, interrupt will be triggered and we check how many 16bits half word (HW) data was received
 * If it is two HWs and the MSB in the first HW is zero, we consider one read command transfer has been received.
 * First HW is read address and second is CRC.
 * If it is 4 HWs and the MSB in the first HW is 1, we consider one write command transfer has been received.
 * First HW is write address, second & third HWs are value, fourth is CRC.
 * No matter witch command has been received, we will copy all data received into on buffer and check CRC in back ground routine.
 *
 * Besides all of this, there is at least one routine communication every 200ms like read battery/current.
 * We will check the successfully communication number every 1 second.
 * If it is not increasing, we will consider something bad has happened and cut off the power of motor.
 */

/*
 * When one read command was received, the application will put the address' data into FIFO of SPI.
 * So next time when master is sending another frame (read/write, address/dummy address), it can retrieve the data last time queried.
 * In general, if you want to read some data,
 * you need send first frame including read command of one valid address, then send another read command to get the data.
 * Because read command is 2 16bits length, it can only access one half word value and one CRC16.
 */

uint16_t CRC16(uint8_t* pData, uint16_t unLength)
{
	uint8_t unCRCHi = 0xFF;
	uint8_t unCRCLo = 0xFF;
    int32_t nIndex;

    while(unLength--)
    {
    	nIndex = unCRCLo ^ (*pData);
        unCRCLo = (uint8_t)(unCRCHi ^ CRC_HIGH_FACTOR[nIndex]);
        unCRCHi = CRC_LOW_FACTOR[nIndex];
    }
    return (uint16_t)(unCRCHi << 8 | unCRCLo);
}

int32_t nReadCommandHandler(uint16_t* pCOM_Buff)
{
	if (COMM_GET_DATA(pCOM_Buff[0]) < COMM_READ_MAX)
	{
		SPI_WRITE_TX(SPI, tMotor.unValue[COMM_GET_DATA(pCOM_Buff[0])]);
		SPI_WRITE_TX(SPI, CRC16((uint8_t*)(&(tMotor.unValue[COMM_GET_DATA(pCOM_Buff[0])])), 1));
		return 0;
	}
	else
	{
		return -1;
	}
}

int32_t nWriteCommandHandler(uint16_t* pCOM_Buff)
{
	switch(COMM_GET_DATA(pCOM_Buff[0]))
	{
	case COMM_WRITE_MOTOR_NEED_TO_RUN:
	case COMM_WRITE_ROTATE_DIRECTION:
		tMotor.structMotor.MCR.bMotorNeedToRun = pCOM_Buff[1];
		break;
	case COMM_WRITE_LOCATING_DUTY:
		tMotor.structMotor.unLocatingDuty = pCOM_Buff[1];
		break;
	case COMM_WRITE_RAMP_UP_DUTY:
		tMotor.structMotor.unRampUpDuty = pCOM_Buff[1];
		break;
	case COMM_WRITE_TARGET_DUTY:
		tMotor.structMotor.unTargetDuty = pCOM_Buff[1];
		break;
	case COMM_WRITE_LOCATING_PERIOD:
		tMotor.structMotor.unLocatingPeriod = pCOM_Buff[1];
		break;
	case COMM_WRITE_RAMP_UP_PERIOD:
		tMotor.structMotor.unRampUpPeriod = pCOM_Buff[1] + pCOM_Buff[2] << 16;
			break;
	default:
		return -1;
	}
	return 0;
}

// Communicating with mast via SPI
void COMM_Manager(void)
{
	static uint16_t unCOM_Buff[COMM_FIFO_LENGTH];
	static uint32_t unLastFrameCNT = 0;
	static uint32_t unLastCheckTime = 0;
	// All transactions are handled in interrupt
	if (tMotor.structMotor.MSR.bNewComFrameReceived == TRUE)
	{
		memcpy(unCOM_Buff, unCOM_SPI_ReadData, COMM_FIFO_LENGTH);
		tMotor.structMotor.MSR.bNewComFrameReceived = FALSE;
		if (CRC16((uint8_t *)unCOM_Buff, (IS_COMM_RD(unCOM_Buff[0]) ? (COMM_RD_CMD_CNT - 1) : (COMM_WR_CMD_CNT - 1))) ==
				(IS_COMM_RD(unCOM_Buff[0]) ? unCOM_Buff[COMM_RD_CMD_CNT - 1] : unCOM_Buff[COMM_WR_CMD_CNT - 1]))
		{
			unValidFrameCNT++;
			// safe zone
			if (IS_COMM_RD(unCOM_Buff[0]))
			{
				nReadCommandHandler(unCOM_Buff);
			}
			else
			{
				nWriteCommandHandler(unCOM_Buff);
			}
		}
		else
		{
			unCOM_SPI_TransErrCNT++;
		}
	}

	// Comm protection 1: If have received some frame in 500ms, error
	if ((uint32_t)(unSystemTick - unLastCheckTime) > 500)
	{
		unLastCheckTime = unSystemTick;
		if ((uint32_t)(unValidFrameCNT - unLastFrameCNT) < 1)
		{
			BLDC_stopMotor();
			setError(ERR_COMMUNICATION_FAIL);
		}
		unLastFrameCNT = unValidFrameCNT;
	}
	// Comm protection 2: If received error frame exceed some threshold, error
	if (unCOM_SPI_TransErrCNT > COM_SPI_TRANS_ERR_THRESHOLD)
	{
		BLDC_stopMotor();
		setError(ERR_COMMUNICATION_FAIL);
	}
}
