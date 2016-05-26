/**
  ******************************************************************************
  * @file    StoreCMXImage.h
  * @brief   Файл, содержащий все необходимое для хранения/чтения образа
  *						микросхемы вокодера CMX7262 в/из EEPROM
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 NTLab
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STORE_CMX_IMAGE_H
#define __STORE_CMX_IMAGE_H

#include "eeprom.h"
#include "globals.h"


#define CMX7262_IMAGE_ADDR_IN_EEPROM	((uint32_t)0x00000000) 

void SaveCMX7262Image();
void ReadCMXImageStruct(void *pStructData);
void ReadPageFromEEPROM(uint16_t noPage, uint16_t* pData);

void ReadWordFromEEPROM(uint32_t addrEEPROM, uint16_t* pDataWord);

#endif /* __STORE_CMX_IMAGE_H */
