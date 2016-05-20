/**
  ******************************************************************************
  * @file    eeprom.h
  * @brief   Файл, содержащий все необходимое для работы с микросхемой памяти
  *						M24M01-RMN6TP типа EEPROM
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 NTLab
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H


#include <string.h>

#ifdef STM32F071xB
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#endif
	 
#include "timers.h"
	 
#define EEPROM_I2C1_ADDR     			(0x50)
#define EEPROM_TEST_INT_MEM_ADDR	(0x1A000)

#define EEPROM_OPERATION_TIMEOUT	(200)

#define EEPROM_SIZE_OF_PAGE				(256)


#define EEPROM_DEV_TYPE_ID				(0xA)
#define EEPROM_CE_ADDR						(0x0)

#define SHIFT_DEV_TYPE_ID					(4)
#define SHIFT_CE_ADDR							(2)
#define SHIFT_ADDR16							(1)


uint8_t WriteToEEPROM(I2C_HandleTypeDef *hi2c1, uint32_t nStartIntAddress, uint8_t *pData, uint32_t nSizeData);
uint8_t ReadFromEEPROM(I2C_HandleTypeDef *hi2c1, uint32_t nStartIntAddress, uint8_t *pData, uint32_t nSizeData);

uint8_t TestEEPROM(I2C_HandleTypeDef *hi2c1);


#endif /* __EEPROM_H */
