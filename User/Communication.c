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

/*
 * The communication works like this:
 * If one unit transfer is finished, interrupt will be triggered and we check how many 16bits half word (HW) data was received
 * If it is two HWs and the MSB in the first HW is zero, we consider one received command transfer has been received.
 * If it is 4 HWs and the MSB in the first HW is 1, we consider one write command transfer has been received.
 * No matter witch command has been received, we will copy all data received into on buffer and check CRC in back ground routine.
 */
// Communicating with mast via SPI
void COMM_Manager(void)
{
	static uint16_t unCOM_Buff[COMM_FIFO_LENGTH];
	// All transactions are handled in interrupt
	if (tMotor.structMotor.MSR.bNewComFrameReceived == TRUE)
	{
		memcpy(unCOM_Buff, unCOM_SPI_ReadData);
		tMotor.structMotor.MSR.bNewComFrameReceived = FALSE;
		if (CRC16(unCOM_Buff) == unCOM_Buff[COMM_FIFO_LENGTH - 1])
		{

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
