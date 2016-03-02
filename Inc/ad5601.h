/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_AD5601_H
#define __SPI_AD5601_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "spi_periphery.h"
#include "timers.h"
	 
	 
	 
uint8_t DAC_write (SPI_HandleTypeDef *hspi);	 
	 
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_AD5601_H */	 
