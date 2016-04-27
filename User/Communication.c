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
 * If it is two HWs and the MSB in the first HW is zero, we consider one received command transfer has been received.
 * If it is 4 HWs and the MSB in the first HW is 1, we consider one write command transfer has been received.
 * No matter witch command has been received, we will copy all data received into on buffer and check CRC in back ground routine.
 *
 * Besides all of this, there is at least one routine communication every 200ms like read battery/current.
 * We will check the successfully communication number every 1 second.
 * If it is not increasing, we will consider something bad has happened and cut off the power of motor.
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

	return 0;
}

// Communicating with mast via SPI
void COMM_Manager(void)
{
	static uint16_t unCOM_Buff[COMM_FIFO_LENGTH];
	// All transactions are handled in interrupt
	if (tMotor.structMotor.MSR.bNewComFrameReceived == TRUE)
	{
		memcpy(unCOM_Buff, unCOM_SPI_ReadData, COMM_FIFO_LENGTH);
		tMotor.structMotor.MSR.bNewComFrameReceived = FALSE;
		if (CRC16((uint8_t *)unCOM_Buff, (IS_COMM_RD(unCOM_Buff[0]) ? (COMM_RD_CMD_CNT - 1) : (COMM_WR_CMD_CNT - 1))) ==
				(IS_COMM_RD(unCOM_Buff[0]) ? unCOM_Buff[COMM_RD_CMD_CNT - 1] : unCOM_Buff[COMM_WR_CMD_CNT - 1]))
		{
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

	if (unCOM_SPI_TransErrCNT > COM_SPI_TRANS_ERR_THRESHOLD)
	{
		setError(ERR_COMMUNICATION_FAIL);
	}
}
