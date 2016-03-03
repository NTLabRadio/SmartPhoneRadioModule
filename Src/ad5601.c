#include "ad5601.h"

// ������� ��������� ��� AD5601 �� ������������ ������������ ���������� ��� ���������� SKY65366
// �������� ���������� �� ������ AD5601 �������������� �������� ������������ ���������� �� ���������: 2.25 �
uint8_t AD5601_SetVCPForSky(SPI_HandleTypeDef *hspi)
{
	uint8_t pAD5601TxData[2];
	uint8_t pAD5601RxData[2];
	
	// ������ ������������� ���������� �� ������ ��� 2.25 �: 
	//((2.25�/3.2�)*2^8)<<4
	pAD5601TxData[0] = 0x2B;
	pAD5601TxData[1] = 0xC0;
	
	// CS ��������
	AD5601_CSN_LOW();
	// ��������� 2 ����� �� spi1
	if (SPI_TransmitRecieve(hspi, pAD5601TxData, pAD5601RxData, 2)) 
	{
			return (0);
	}
	// CS �������
	AD5601_CSN_HIGH();

	return 1;		
}
