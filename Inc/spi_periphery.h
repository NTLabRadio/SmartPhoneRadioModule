/**
  ******************************************************************************
  * @file    spi_periphery.h
  * @brief   Файл, содержащий все необходимое для работы с периферийными
	*					 устройствами макета радиомодуля для смартфона на базе
	*					 SDR-демонстратора DE9943
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 NTLab
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_PERIPHERY_H
#define __SPI_PERIPHERY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "globals.h"
#include "timers.h"
	 
typedef enum
{
  DEVICE_CMX7262      = 0x00,
  DEVICE_CC1120      	= 0x01,
	NUM_OF_PERIPH_DEVICES
} HandlePeriphDeviceTypeDef;

typedef enum
{
	INTERFACE_SPI1,
	INTERFACE_SPI2,
	NUM_OF_SPI_INTERFACES
} InterfaceSPITypeDef;


// Definitions for IO port mapped as chip select (output).

#define AD5601_SPI_CS_PIN              	SPI1_CS_AD5601_Pin         	/* PA.04 */
#define AD5601_SPI_CS_GPIO_PORT        	SPI1_CS_AD5601_GPIO_Port   	/* GPIOA */


// Definitions for IO port mapped as chip select (output).

#define CMX7262_SPI_CS_PIN              	GPIO_PIN_2                 	/* PA.02 */
#define CMX7262_SPI_CS_GPIO_PORT        	GPIOA                      	/* GPIOA */

// GPIO lines for SPI interfaces

#define CMX7262_SPI_SCK_PIN               GPIO_PIN_5                  /* PA.05 */
#define CMX7262_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */

#define CMX7262_SPI_MISO_PIN              GPIO_PIN_6                  /* PA.06 */
#define CMX7262_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */

#define CMX7262_SPI_MOSI_PIN              GPIO_PIN_7                  /* PA.07 */
#define CMX7262_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */

//IRQ from CMX7262
#define CMX7262_IRQN_PIN              		GPIO_PIN_1                  /* PA.01 */
#define CMX7262_IRQN_PORT        					GPIOA                       /* GPIOA */


// Definitions for IO port mapped as chip select (output).

#define CC1120_SPI_CS_PIN              	SPI2_CS_CC1120_Pin         	/* PB.12 */
#define CC1120_SPI_CS_GPIO_PORT        	SPI2_CS_CC1120_GPIO_Port  	/* GPIOB */

// GPIO lines for SPI interfaces

#define CC1120_SPI_SCK_PIN               	GPIO_PIN_13                 /* PB.13 */
#define CC1120_SPI_SCK_GPIO_PORT         	GPIOB                       /* GPIOB */

#define CC1120_SPI_MISO_PIN              	GPIO_PIN_14                 /* PB.14 */
#define CC1120_SPI_MISO_GPIO_PORT        	GPIOB                       /* GPIOB */

#define CC1120_SPI_MOSI_PIN              	GPIO_PIN_15                 /* PB.15 */
#define CC1120_SPI_MOSI_GPIO_PORT        	GPIOB                       /* GPIOB */

//IRQ from CC1120
#define CC1120_IRQN_PIN              			IRQ_CC1120_Pin              /* PB.00 */
#define CC1120_IRQN_PORT        					IRQ_CC1120_GPIO_Port        /* GPIOB */

#define CMX7262_CSN_LOW()     HAL_GPIO_WritePin(CMX7262_SPI_CS_GPIO_PORT, CMX7262_SPI_CS_PIN, GPIO_PIN_RESET)
#define CMX7262_CSN_HIGH()    HAL_GPIO_WritePin(CMX7262_SPI_CS_GPIO_PORT, CMX7262_SPI_CS_PIN, GPIO_PIN_SET)
#define CC1120_CSN_LOW()     	HAL_GPIO_WritePin(CC1120_SPI_CS_GPIO_PORT, CC1120_SPI_CS_PIN, GPIO_PIN_RESET)
#define CC1120_CSN_HIGH()			HAL_GPIO_WritePin(CC1120_SPI_CS_GPIO_PORT, CC1120_SPI_CS_PIN, GPIO_PIN_SET)
#define AD5601_CSN_LOW()      HAL_GPIO_WritePin(SPI1_CS_AD5601_GPIO_Port, SPI1_CS_AD5601_Pin, GPIO_PIN_RESET)
#define AD5601_CSN_HIGH()     HAL_GPIO_WritePin(SPI1_CS_AD5601_GPIO_Port, SPI1_CS_AD5601_Pin, GPIO_PIN_SET)

	 
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

void ResetCpltState_SPI_TransmitReceive(SPI_HandleTypeDef *hspi);
uint8_t isCplt_SPI_TransmitReceive(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef SPI_TransmitRecieveByte(SPI_HandleTypeDef *hspi, uint8_t nByteForTX, uint8_t *nByteForRX);
HAL_StatusTypeDef SPI_TransmitRecieve(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void SPI_TIMEOUT_UserCallback(SPI_HandleTypeDef *hspi);
	 	 
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_PERIPHERY_H */
