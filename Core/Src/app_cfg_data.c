/*
 * app_cfg_data.c
 *
 *  Created on: Mar 17, 2024
 *      Author: SoftwareEngineer_01
 */
#include <string.h>
#include "main.h"
#include "tools.h"
#include "app_cfg_data.h"

/* FLASH_PAGE_SIZE should be able to get the size of the Page according to the controller */
static uint32_t GetPage(uint32_t Address)
{
  for (int indx=0; indx<128; indx++)
  {
	  if((Address < (0x08000000 + (FLASH_PAGE_SIZE *(indx+1))) ) && (Address >= (0x08000000 + FLASH_PAGE_SIZE*indx)))
	  {
		  return (0x08000000 + FLASH_PAGE_SIZE*indx);
	  }
  }

  return 0;
}

static uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	  uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords*4;
	  uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.PageAddress = StartPage;
	   EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}

static uint32_t Flash_Write_HalfWordData (uint32_t StartPageAddress, uint16_t *Data, uint16_t numberofhwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	  uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofhwords*2;
	  uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.PageAddress = StartPage;
	   EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofhwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 2;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}

static void Flash_Read_HalfWordData (uint32_t StartPageAddress, uint16_t *RxBuf, uint16_t numberofhwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint16_t *)StartPageAddress;
		StartPageAddress += 2;
		RxBuf++;
		if (!(numberofhwords--)) break;
	}
}

static void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}


uint8_t app_cfg_save(uint8_t* cfg_buf, uint16_t len) {//app_cfg_t *app_cfg) {
	app_cfg_t cfg;
	cfg.data = &cfg_buf[2];

	// размер cfg без учета CRC
	uint16_t cfgHWordNum = (len >> 1) + 1;
	uint16_t crc = Crc16((uint8_t*)cfg.data, len - 2);
	*(uint16_t*)cfg_buf = crc;

	Flash_Write_HalfWordData(APP_CFG_FLASH_ADDRESS, (uint16_t*)cfg_buf, cfgHWordNum);

	return 1;
}

ErrorStatus app_settings_load(app_device_settings_t *data) {

	Flash_Read_HalfWordData(APP_CFG_FLASH_ADDRESS, (uint16_t*)data, (sizeof(app_device_settings_t) >> 1));

	// размер cfg без учета CRC
	uint16_t crc = Crc16((uint8_t*)&data->MB_ADDRESS, sizeof(app_device_settings_t) - sizeof(data->CRC_16));

	if(crc == data->CRC_16)
		return SUCCESS;
	else
		return !SUCCESS;
}

ErrorStatus servo_cfg_load(app_servo_cfg_t *data, uint8_t servo_id) {

	uint16_t servosCnt = 0, memId = UINT16_MAX;
	uint32_t mem_offset = 0;

	// чтение памяти для поиска нужного id
	while(servosCnt < 32) {
		Flash_Read_HalfWordData(SERVO_CFG_FLASH_ADDRESS + mem_offset, (uint16_t*)&memId, 1);

		// Crc8 в первом байтие, Id во втором байте
		memId >>= 8;

		if((uint8_t)((memId) & 0xff) == servo_id)
			break;
		else {
			servosCnt++;
			mem_offset += sizeof(app_servo_cfg_t);
		}
	}

	// Если ID не найден вернуть ошибку
	if(servosCnt > 32 || memId == UINT16_MAX)
		return !SUCCESS;

	// чтение памяти настроек
	Flash_Read_HalfWordData(APP_CFG_FLASH_ADDRESS + mem_offset, (uint16_t*)data, (sizeof(app_servo_cfg_t) >> 1));
	uint8_t crc = Crc16(&data->servo_id, sizeof(app_servo_cfg_t) - sizeof(data->crc_8));

	if(crc == data->crc_8)
		return SUCCESS;
	else
		return !SUCCESS;
}

uint8_t app_cfg_load(uint8_t* cfg_buf, uint16_t len) {
	app_cfg_t cfg;
	cfg.data = cfg_buf;
	uint16_t cfgHalfWordNum = (len >> 1) + 1;

	// читаем CRC
	Flash_Read_HalfWordData(APP_CFG_FLASH_ADDRESS, (uint16_t*)&cfg.CRC_16, 1);
	Flash_Read_HalfWordData(APP_CFG_FLASH_ADDRESS + 2, (uint16_t*)cfg.data, cfgHalfWordNum);

	// размер cfg без учета CRC
	uint16_t crc = Crc16((uint8_t*)cfg.data, len);

	if(cfg.CRC_16 == crc)
		return 1;
	else
		memset(cfg.data, 0, len);

	return 0;
}
