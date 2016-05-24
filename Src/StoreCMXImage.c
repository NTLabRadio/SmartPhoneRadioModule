#include "StoreCMXImage.h"

extern I2C_HandleTypeDef hi2c1;

/**
  * @brief  Функция записи образа CMX7262 из памяти ARM в EEPROM
  *
	* @note   Для записи образа CMX7262 в EEPROM он должен быть 
	*					предварительно размещен каким-либо образом во внутренней
	*					памяти ARM (это может быть память загрузочной флеш). 
	*					Для этого должно быть выделено пространство размером
	*					MAX_SIZE_OF_CMX7262_IMAGE байт (92 кБайт)
	*					В EEPROM образ должен быть записан целиком: структура образа
	*					и 2 блока памяти.
	* @note		Функция SaveCMX7262Image() читает из памяти ARM не точный
	*					размер образа, а максимальный объем памяти, отводимый	под 
	*					образ (MAX_SIZE_OF_CMX7262_IMAGE байт)
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
  * @brief  Функция чтения структуры образа CMX7262 из EEPROM
  *
  * @param	pStructData - указатель для возврата данных структуры;
  *					память для структуры должна быть выделена вызывающей функцией
	*/
void ReadCMXImageStruct(void *pStructData)
{
	const uint32_t nSizeData = sizeof(cmxFI_TypeDef);
	
	ReadFromEEPROM(&hi2c1, CMX7262_IMAGE_ADDR_IN_EEPROM, (uint8_t*)pStructData, nSizeData);
}


/**
  * @brief  Функция чтения страницы памяти из EEPROM
	*
	* @param  noPage - номер читаемой страницы;
  * @param	pData - указатель для возврата данных страницы,
	*					перед вызовом для него должна быть выделена память
	*					размером EEPROM_SIZE_OF_PAGE байт
	*/
void ReadPageFromEEPROM(uint16_t noPage, uint16_t* pData)
{
	uint32_t addrPageEEPROM = (uint32_t)noPage*EEPROM_SIZE_OF_PAGE;
	ReadFromEEPROM(&hi2c1, addrPageEEPROM, (uint8_t*)pData, EEPROM_SIZE_OF_PAGE);

}

/**
  * @brief  Функция чтения слова данных из EEPROM
	*
	* @param  addrEEPROM - адрес читаемого слова в EEPROM;
  * @param	nDataWord - указатель для возврата значения слова
	*
	* @note   Функция оптимизирована для пословного чтения 
  *					больших неразрывных областей памяти. 
	*					При чтении слова кэшируются данные целой страницы 
  *					памяти EEPROM, на которой находится слово. При 
	*					следующей операции чтения проверяется, входит
	*					ли искомое слово в уже прочитанную, кэшированную
	*					страницу. Если входит, то операция чтения из EEPROM
	*					не выполняется, данные читаются из кэш
  */
void ReadWordFromEEPROM(uint32_t addrEEPROM, uint16_t* nDataWord)
{
	static uint16_t prevNoEEPROMPage = 0xFFFF;
	uint16_t curNoEEPROMPage;
	uint16_t pPageData[EEPROM_SIZE_OF_PAGE];
	
	curNoEEPROMPage = addrEEPROM/EEPROM_SIZE_OF_PAGE;
	if(curNoEEPROMPage!=prevNoEEPROMPage)
	{
		//читаем новую страницу EEPROM
		ReadPageFromEEPROM(curNoEEPROMPage, pPageData);
		prevNoEEPROMPage = curNoEEPROMPage;
	}

	//читаем не из EEPROM, а уже из ранее прочитанной страницы
	uint16_t addrWordInPage = (addrEEPROM%EEPROM_SIZE_OF_PAGE)/sizeof(uint16_t);
	*nDataWord = pPageData[addrWordInPage];
}
