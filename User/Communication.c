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

// Communicating with mast via SPI
void CommunicationManager(void)
{
	// All transactions are handled in interrupt
	if (tMotor.structMotor.MSR.bNewComFrameReceived == TRUE)
	{
		tMotor.structMotor.MSR.bNewComFrameReceived = FALSE;

	}
}
