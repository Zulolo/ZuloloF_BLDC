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

uint16_t calCRC16(uint8_t* pBytes, uint32_t unLength)
{
	uint16_t crc = 0;
	uint32_t unIndex;
	uint8_t unPosInTable;
	
	for (unIndex = 0; unIndex < unLength; unIndex++)
	{
		/* XOR-in next input byte into MSB of crc, that's our new intermediate divident */
		unPosInTable = (uint8_t)((crc >> 8) ^ (*(pBytes + REVS_BYTE_ORDER(unIndex)))); /* equal: ((crc ^ (b << 8)) >> 8) */
		/* Shift out the MSB used for division per lookuptable and XOR with the remainder */
		crc = (uint16_t)((crc << 8) ^ (uint16_t)(CRC_TABLE16[unPosInTable]));
	}

	return crc;
}

int32_t nReadCommandHandler(uint16_t unReadCommand)
{
	static uint8_t unRegSelect;
	static uint16_t unReadValue;	// tMotor.unValue may be changed in interrupt between SPI_WRITE_TX and calculating CRC
	unRegSelect = COMM_GET_DATA(unReadCommand);
	if (unRegSelect < MOTOR_READ_MAX)
	{
		unReadValue = tMotor.unValue[unRegSelect];
		SPI_WRITE_TX(SPI, unReadValue);
		unReadValueCRC = calCRC16((uint8_t *)(&unReadValue), 2);
		return 0;
	}
	else
	{
		unReadValueCRC = 0;
		return -1;
	}
}

int32_t nWriteCommandHandler(uint16_t* pCOM_Buff)
{
	switch(COMM_GET_DATA(pCOM_Buff[0]))
	{
	case MOTOR_WRITE_MOTOR_NEED_TO_RUN:
	case MOTOR_WRITE_ROTATE_DIRECTION:
		tMotor.structMotor.MCR.bMotorNeedToRun = pCOM_Buff[1];
		break;
	case MOTOR_WRITE_LOCATING_DUTY:
		tMotor.structMotor.unLocatingDuty = pCOM_Buff[1];
		break;
	case MOTOR_WRITE_RAMP_UP_DUTY:
		tMotor.structMotor.unRampUpDuty = pCOM_Buff[1];
		break;
	case MOTOR_WRITE_TARGET_DUTY:
		tMotor.structMotor.unTargetDuty = pCOM_Buff[1];
		break;
	case MOTOR_WRITE_LOCATING_PERIOD:
		tMotor.structMotor.unLocatingPeriod = pCOM_Buff[1];
		break;
	case MOTOR_WRITE_RAMP_UP_PERIOD_LOW:
		tMotor.structMotor.unRampUpPeriod = (tMotor.structMotor.unRampUpPeriod & LOW_REG_CLR_MASK) | ((uint32_t)(pCOM_Buff[1]));
		break;
	case MOTOR_WRITE_RAMP_UP_PERIOD_HIGH:
		tMotor.structMotor.unRampUpPeriod = (tMotor.structMotor.unRampUpPeriod & HIGH_REG_CLR_MASK) | (((uint32_t)(pCOM_Buff[1])) << 16);
		break;
	default:

		return -1;
	}

	return 0;
}

// Communicating with mast via SPI

void COMM_Manager(void)
{
	static uint32_t unLastFrameCNT = 0;
	static uint32_t unLastCheckTime = 0;
	static ENUM_SPI_RECEIVE_STATE tSPI_LastState = SPI_RCV_IDLE;
	static uint16_t unSPI_RX_Value;
	static uint16_t unCOM_SPI_ReadData[4];	//COMM_FIFO_LENGTH];	// 0 or 0xFFFF means no data
	
	// All transactions are handled in interrupt
	if (tMotor.structMotor.MSR.bNewComFrameReceived == TRUE)
	{
		tMotor.structMotor.MSR.bNewComFrameReceived = FALSE;	
		unSPI_RX_Value = SPI_READ_RX(SPI);
		
		switch(tSPI_LastState)
		{	
			case SPI_RCV_IDLE:
				if ((MTR_INVALID_MOTOR_CMD == unSPI_RX_Value) || (MTR_NULL_MOTOR_CMD == unSPI_RX_Value))
				{
					SPI_WRITE_TX(SPI, 0);
					tSPI_LastState = SPI_RCV_IDLE;
				}
				else
				{
					unCOM_SPI_ReadData[0] = unSPI_RX_Value;
					if (IS_COMM_RD_CMD(unSPI_RX_Value))
					{
						tSPI_LastState = SPI_RCV_RD_CMD;
					}
					else
					{
						tSPI_LastState = SPI_RCV_WR_CMD;
					}						
				}						
			break;
			
			case SPI_RCV_CRC:
				if (MTR_INVALID_MOTOR_CMD == unSPI_RX_Value)
				{	// After read command and CRC
					SPI_WRITE_TX(SPI, unReadValueCRC);
					tSPI_LastState = SPI_RCV_IDLE;
				}
				else if (IS_COMM_RD_CMD(unSPI_RX_Value))
				{
					// write command after write CRC
					unCOM_SPI_ReadData[0] = unSPI_RX_Value;
					tSPI_LastState = SPI_RCV_WR_CMD;
				}
				else
				{
					tMotor.structMotor.unCommErrCNT++;
					tSPI_LastState = SPI_RCV_IDLE;									
				}	
			break;
		
			case SPI_RCV_RD_CMD:
				// If last time is read command, this time must be read CRC and next time must be 0xFFFF on MOSI to read
				// So the data received is the CRC of read command, now the slave doesn't care
				unCOM_SPI_ReadData[1] = unSPI_RX_Value;
				if (nReadCommandHandler(unCOM_SPI_ReadData[0]) == 0)
				{
					tMotor.structMotor.unCommOK_CNT++;
				}				
				tSPI_LastState = SPI_RCV_CRC;	
			break;

			case SPI_RCV_WR_CMD:
				// No need to comment
//					SPI_WRITE_TX(SPI, 0);
				unCOM_SPI_ReadData[1] = unSPI_RX_Value;
				tSPI_LastState = SPI_RCV_WR_DATA;	
			break;
			
			case SPI_RCV_WR_DATA:
				// No need to comment
				unCOM_SPI_ReadData[2] = unSPI_RX_Value;
				if (calCRC16((uint8_t *)unCOM_SPI_ReadData, 4) == unCOM_SPI_ReadData[2])
				{
					if (nWriteCommandHandler(unCOM_SPI_ReadData) == 0)
					{
						tMotor.structMotor.unCommOK_CNT++;
					}
					else
					{
//						unCOM_SPI_TransErrCNT++;
					}
				}
				tSPI_LastState = SPI_RCV_CRC;	
			break;
		
			default:
				tMotor.structMotor.unCommErrCNT++;
			break;
		}
		SPI_TRIGGER(SPI);
	}
	
	// Comm protection 1: If have NOT received any frame in 500ms, error
	if ((uint32_t)(unSystemTick - unLastCheckTime) > 500)
	{
		unLastCheckTime = unSystemTick;
		if ((uint32_t)(tMotor.structMotor.unCommOK_CNT - unLastFrameCNT) < 1)
		{
			BLDC_stopMotor();
			setError(ERR_COMMUNICATION_FAIL);
			// I don't know why,
			// But seems every time enter here, the GO_BUSY bit of SPI Control register will be reset
			SPI_TRIGGER(SPI);			
		}
		unLastFrameCNT = tMotor.structMotor.unCommOK_CNT;
	}
	// Comm protection 2: If received error frame exceed some threshold, error
	if (tMotor.structMotor.unCommErrCNT > COM_SPI_TRANS_ERR_THRESHOLD)
	{
		BLDC_stopMotor();
		setError(ERR_COMMUNICATION_FAIL);
		SPI_TRIGGER(SPI);	
	}
}
