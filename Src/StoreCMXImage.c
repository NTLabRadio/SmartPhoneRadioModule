#include "StoreCMXImage.h"

extern I2C_HandleTypeDef hi2c1;

/**
  * @brief  ������� ������ ������ CMX7262 �� ������ ARM � EEPROM
  *
	* @note   ��� ������ ������ CMX7262 � EEPROM �� ������ ���� 
	*					�������������� �������� �����-���� ������� �� ����������
	*					������ ARM (��� ����� ���� ������ ����������� ����). 
	*					��� ����� ������ ���� �������� ������������ ��������
	*					MAX_SIZE_OF_CMX7262_IMAGE ���� (92 �����)
	*					� EEPROM ����� ������ ���� ������� �������: ��������� ������
	*					� 2 ����� ������.
	* @note		������� SaveCMX7262Image() ������ �� ������ ARM �� ������
	*					������ ������, � ������������ ����� ������, ���������	��� 
	*					����� (MAX_SIZE_OF_CMX7262_IMAGE ����)
	*/
void SaveCMX7262Image()
{
	#ifdef CMX7262_IMAGE_IN_FLASH
	uint8_t *pImageData = (uint8_t *)CMX7262_IMAGE_ADDR;
	const uint32_t nSizeData = MAX_SIZE_OF_CMX7262_IMAGE;
	
	WriteToEEPROM(&hi2c1, CMX7262_IMAGE_ADDR_IN_EEPROM, pImageData, nSizeData);
	#endif
}

/**
  * @brief  ������� ������ ��������� ������ CMX7262 �� EEPROM
  *
  * @param	pStructData - ��������� ��� �������� ������ ���������;
  *					������ ��� ��������� ������ ���� �������� ���������� ��������
	*/
void ReadCMXImageStruct(void *pStructData)
{
	const uint32_t nSizeData = sizeof(cmxFI_TypeDef);
	
	ReadFromEEPROM(&hi2c1, CMX7262_IMAGE_ADDR_IN_EEPROM, (uint8_t*)pStructData, nSizeData);
}


/**
  * @brief  ������� ������ �������� ������ �� EEPROM
	*
	* @param  noPage - ����� �������� ��������;
  * @param	pData - ��������� ��� �������� ������ ��������,
	*					����� ������� ��� ���� ������ ���� �������� ������
	*					�������� EEPROM_SIZE_OF_PAGE ����
	*/
void ReadPageFromEEPROM(uint16_t noPage, uint16_t* pData)
{
	uint32_t addrPageEEPROM = (uint32_t)noPage*EEPROM_SIZE_OF_PAGE;
	ReadFromEEPROM(&hi2c1, addrPageEEPROM, (uint8_t*)pData, EEPROM_SIZE_OF_PAGE);

}

/**
  * @brief  ������� ������ ����� ������ �� EEPROM
	*
	* @param  addrEEPROM - ����� ��������� ����� � EEPROM;
  * @param	nDataWord - ��������� ��� �������� �������� �����
	*
	* @note   ������� �������������� ��� ���������� ������ 
  *					������� ����������� �������� ������. 
	*					��� ������ ����� ���������� ������ ����� �������� 
  *					������ EEPROM, �� ������� ��������� �����. ��� 
	*					��������� �������� ������ �����������, ������
	*					�� ������� ����� � ��� �����������, ������������
	*					��������. ���� ������, �� �������� ������ �� EEPROM
	*					�� �����������, ������ �������� �� ���
  */
void ReadWordFromEEPROM(uint32_t addrEEPROM, uint16_t* nDataWord)
{
	static uint16_t prevNoEEPROMPage = 0xFFFF;
	uint16_t curNoEEPROMPage;
	uint16_t pPageData[EEPROM_SIZE_OF_PAGE];
	
	curNoEEPROMPage = addrEEPROM/EEPROM_SIZE_OF_PAGE;
	if(curNoEEPROMPage!=prevNoEEPROMPage)
	{
		//������ ����� �������� EEPROM
		ReadPageFromEEPROM(curNoEEPROMPage, pPageData);
		prevNoEEPROMPage = curNoEEPROMPage;
	}

	//������ �� �� EEPROM, � ��� �� ����� ����������� ��������
	uint16_t addrWordInPage = (addrEEPROM%EEPROM_SIZE_OF_PAGE)/sizeof(uint16_t);
	*nDataWord = pPageData[addrWordInPage];
}
