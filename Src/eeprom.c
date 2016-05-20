#include "eeprom.h"


uint8_t TestEEPROM(I2C_HandleTypeDef *hi2c1)
{
	uint8_t nRes;
	
	HAL_StatusTypeDef status;
  status = HAL_I2C_IsDeviceReady(hi2c1, (uint16_t)(EEPROM_I2C1_ADDR<<1), 10, EEPROM_OPERATION_TIMEOUT);

	if(status!=HAL_OK)
		return(0);

	const uint16_t nSizeXBuffer = 4;
	uint8_t xBufferTx[nSizeXBuffer] = {0x46, 0x33, 0x28, 0x97};
	uint8_t xBufferRx[nSizeXBuffer] = {0,0,0,0};

	nRes = WriteToEEPROM(hi2c1, EEPROM_TEST_INT_MEM_ADDR, xBufferTx, nSizeXBuffer);
	
	if(!nRes)
		return(0);
	
	//WaitTimeMCS(4e3); //����
	WaitTimeMCS(5e3);

	nRes = ReadFromEEPROM(hi2c1, EEPROM_TEST_INT_MEM_ADDR, xBufferRx, nSizeXBuffer);

	if(!nRes)
		return(0);
	
	if(!memcmp(xBufferTx,xBufferRx,nSizeXBuffer))
		nRes = 1;
	else
		nRes = 0;

	return(nRes);
}


uint8_t WriteToEEPROM(I2C_HandleTypeDef *hi2c1, uint32_t nStartIntAddress, uint8_t *pData, uint32_t nSizeData)
{
	HAL_StatusTypeDef nTransStatus;	//������ (������� ����������) ��������� ����������
	
	nStartIntAddress &= 0x1FFFF;
	
	uint16_t noFirstPage = nStartIntAddress / EEPROM_SIZE_OF_PAGE;
	uint16_t noLastPage = (nStartIntAddress+nSizeData-1) / EEPROM_SIZE_OF_PAGE;
	
	uint16_t nDevSelCode;		//�������� Device Select Code (������ ���� ���������� I2C)
	uint16_t noPage;				//����� ������ �������� EEPROM
	uint16_t nSizePageData;	//������ ������, ������� ������ ���� �������� �� ������� �������� EEPROM, ����
	uint16_t numBytesToEndOfPage;	//����� ���� �� �������� ������ �� ����� ��������

	uint32_t nIntAddress = nStartIntAddress;	//������� ���������� ����� EEPROM
	uint32_t nSizeRemData = nSizeData;				//������ ������, ������� �������� �������������, ����
	
	for(noPage=noFirstPage; noPage<noLastPage+1; noPage++)
	{
		nDevSelCode = (EEPROM_DEV_TYPE_ID<<SHIFT_DEV_TYPE_ID) | (EEPROM_CE_ADDR<<SHIFT_CE_ADDR) | ((nIntAddress>>16)<<SHIFT_ADDR16);
	
		//���������� ����� ����, ������� ���������� �������� �� ������� ��������
		if(noPage==noFirstPage)
		{
			numBytesToEndOfPage = EEPROM_SIZE_OF_PAGE - (nIntAddress%EEPROM_SIZE_OF_PAGE);
			nSizePageData = (nSizeRemData<numBytesToEndOfPage ? nSizeRemData : numBytesToEndOfPage);
		}
		else
			if(noPage==noLastPage)
				nSizePageData = nSizeRemData;	
			else
				nSizePageData = EEPROM_SIZE_OF_PAGE;

		//����� ���, ��� ������ ��������� �� ������� ��������
		nTransStatus = HAL_I2C_Mem_Write(hi2c1, nDevSelCode, nIntAddress&0xFFFF, I2C_MEMADD_SIZE_16BIT, 
																pData+(nSizeData-nSizeRemData), nSizePageData, EEPROM_OPERATION_TIMEOUT);

		if(nTransStatus!=HAL_OK)
			return(0);
			
		nSizeRemData-=nSizePageData;
		nIntAddress+=nSizePageData;
		
		WaitTimeMCS(5e3);
	}
	
	return(1);
}


uint8_t ReadFromEEPROM(I2C_HandleTypeDef *hi2c1, uint32_t nStartIntAddress, uint8_t *pData, uint32_t nSizeData)
{
	HAL_StatusTypeDef nTransStatus;	//������ (������� ����������) ��������� ����������
	
	nStartIntAddress &= 0x1FFFF;
	
	uint16_t noFirstPage = nStartIntAddress / EEPROM_SIZE_OF_PAGE;
	uint16_t noLastPage = (nStartIntAddress+nSizeData) / EEPROM_SIZE_OF_PAGE;
	
	uint16_t nDevSelCode;		//�������� Device Select Code (������ ���� ���������� I2C)
	uint16_t noPage;				//����� ������ �������� EEPROM
	uint16_t nSizePageData;	//������ ������, ������� ������ ���� ��������� � ������� �������� EEPROM, ����
	uint16_t numBytesToEndOfPage;	//����� ���� �� �������� ������ �� ����� ��������

	uint32_t nIntAddress = nStartIntAddress;	//������� ���������� ����� EEPROM
	uint32_t nSizeRemData = nSizeData;				//������ ������, ������� �������� ��������������, ����
	
	for(noPage=noFirstPage; noPage<noLastPage+1; noPage++)
	{
		nDevSelCode = (EEPROM_DEV_TYPE_ID<<SHIFT_DEV_TYPE_ID) | (EEPROM_CE_ADDR<<SHIFT_CE_ADDR) | ((nIntAddress>>16)<<SHIFT_ADDR16);
	
		//���������� ����� ����, ������� ���������� ��������� � ������� ��������
		if(noPage==noFirstPage)
		{
			numBytesToEndOfPage = EEPROM_SIZE_OF_PAGE - (nIntAddress%EEPROM_SIZE_OF_PAGE);
			nSizePageData = (nSizeRemData<numBytesToEndOfPage ? nSizeRemData : numBytesToEndOfPage);
		}
		else
			if(noPage==noLastPage)
				nSizePageData = nSizeRemData;	
			else
				nSizePageData = EEPROM_SIZE_OF_PAGE;		
	
		//������ ���, ��� ������ ��������� � ������� ��������
		nTransStatus = HAL_I2C_Mem_Read(hi2c1, nDevSelCode, nIntAddress&0xFFFF, I2C_MEMADD_SIZE_16BIT, 
																		pData+(nSizeData-nSizeRemData), nSizePageData, EEPROM_OPERATION_TIMEOUT);

		if(nTransStatus!=HAL_OK)
			return(0);
		
		nSizeRemData-=nSizePageData;
		nIntAddress+=nSizePageData;
	}
	
	return(1);
}
