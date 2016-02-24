/****************************************************************************
**
** Copyright (C) 2016 "NTLab"
**
** ���� ���� ��������� ������� ���������� ����������� ���������� SPIM (Smart Phone 
** InterModule)
**
****************************************************************************/

#ifndef SPIMLOGIC_H
#define SPIMLOGIC_H

#include <stdint.h>
#include <string.h>
#include "RadioModule.h"
#include "SPIMMessage.h"
#include "uart_intermodule.h"

extern uint8_t pUARTRxSLIPPack[];
extern uint16_t nSizeSLIPPack;
extern RadioModule g_RadioModule;

enum en_SPIMaddrs
{
	SPIM_ADDR_STM32									=0x1,		//���������� STM32 �������� ���������� (����������)
	SPIM_ADDR_EXTDEV								=0x2		//������� ����������� ���������� (��������� NT1004, �� ��� ��.)
};

enum en_SPIMReqTypes
{
	SPIM_REQTYPE_SINGLE							=0,			//��������� ���������� ������ - ������, ����� �� ������� ������ ���� ������ ������� 
																					//����������� ���������� � ������ ��������� �������
	SPIM_REQTYPE_ASYNC							=1			//����������� ������ - ������, � ����� �� ������� �������� (��������, RSSI) ���������� 
																					//������� ����������� � ������������ ������ �������, ��� ��������� ��� ��������
};

void SPIMInit(void);
void SPIMDeInit(void);

void ProcessDataFromExtDev(void);
void FormAndSendAnswerToExtDev(SPIMMessage* SPIMmsgRcvd);
void FormAnswerToExtDev(SPIMMessage* SPIMCmdRcvd, SPIMMessage* SPIMBackCmdToSend);
void FormBodyOfAnswerToExtDev(SPIMMessage* SPIMCmdRcvd, uint8_t* pBodyData, uint8_t& bodySize);
void FormCurrentParamAnswer(SPIMMessage* SPIMCmdRcvd, uint8_t* pBodyData, uint8_t& bodySize);


#endif // SPIMLOGIC_H
