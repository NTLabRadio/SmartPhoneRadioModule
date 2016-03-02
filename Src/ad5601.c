#include "ad5601.h"

uint8_t pAD5601TxData[2];
uint8_t pAD5601RxData[2];


uint8_t DAC_write (SPI_HandleTypeDef *hspi)
{
	pAD5601TxData[0] = 0x3F;
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
