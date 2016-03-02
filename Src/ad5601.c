#include "ad5601.h"

uint8_t pAD5601TxData[2];
uint8_t pAD5601RxData[2];


uint8_t DAC_write (SPI_HandleTypeDef *hspi)
{
	pAD5601TxData[0] = 0x3F;
	pAD5601TxData[1] = 0xC0;
	
	
	// CS опустить
	AD5601_CSN_LOW();
	
	// отправить 2 байта по spi1
	if (SPI_TransmitRecieve(hspi, pAD5601TxData, pAD5601RxData, 2)) 
	{
			return (0);
	}
	
	// CS поднять
	AD5601_CSN_HIGH();
	
return 1;	
	
}
