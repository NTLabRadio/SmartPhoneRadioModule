#include "ad5601.h"

// Функция настройки ЦАП AD5601 на формирование управляющего напряжения для микросхемы SKY65366
// Значение напряжения на выходе AD5601 сооответствует значению управляющего напряжения по умолчанию: 2.25 В
uint8_t AD5601_SetVCPForSky(SPI_HandleTypeDef *hspi)
{
	uint8_t pAD5601TxData[2];
	uint8_t pAD5601RxData[2];
	
	// Данные соответствуют напряжению на выходе ЦАП 2.25 В: 
	//((2.25В/3.2В)*2^8)<<4
	pAD5601TxData[0] = 0x2B;
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
