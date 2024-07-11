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

static ErrorStatus app_settings_load(app_device_settings_t *data);
static ErrorStatus servo_cfg_load(app_servo_cfg_t *data, uint8_t cfg_num);


static ErrorStatus app_settings_save(app_device_settings_t *data);
static ErrorStatus servo_cfg_save(app_servo_cfg_t *data, uint8_t cfg_num);

app_cfg_data_t app_cfg_data;


uint8_t app_cfg_save(app_cfg_data_t* cfg_buf) {

	app_settings_save(&cfg_buf->devSettings);
	//servo_cfg_save(cfg_buf->servoCfg, 32);

	return 1;
}

uint8_t app_cfg_load(app_cfg_data_t* cfg_buf) {
	app_settings_load(&cfg_buf->devSettings);
	servo_cfg_load(cfg_buf->servoCfg, 32);

	return 0;
}


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

static uint32_t Flash_Add_HalfWordData (uint32_t StartPageAddress, uint16_t *Data, uint16_t numberofhwords)
{
	int sofar=0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

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



static ErrorStatus app_settings_save(app_device_settings_t *data) {
	// 1 - get CRC16
	// 2 -

	uint16_t cfgHWordNum = sizeof(app_device_settings_t) >> 1;
	uint16_t tmpCrc16 = Crc16((uint8_t*)&data->MB_ADDRESS, sizeof(app_device_settings_t) - sizeof(data->CRC_16));
	data->CRC_16 = tmpCrc16;

	uint32_t res = Flash_Write_HalfWordData(APP_CFG_FLASH_ADDRESS, (uint16_t*)data, cfgHWordNum);

	if(res)
		return !SUCCESS;
	else
		return SUCCESS;
}

static ErrorStatus servo_cfg_save(app_servo_cfg_t *data, uint8_t cfg_num) {
	// 1 - get CRC16
	// 2 -
	uint32_t mem_offset = 0, res = 0;

	for(uint16_t cnt = 0; cnt < cfg_num; cnt++) {
		uint16_t cfgHWordNum = sizeof(app_servo_cfg_t) >> 1;
		uint16_t tmpCrc8 = Crc8(&data->servo_id, sizeof(app_servo_cfg_t) - sizeof(data->crc_8));
		data->crc_8 = tmpCrc8;

		res |= Flash_Add_HalfWordData(SERVO_CFG_FLASH_ADDRESS + mem_offset, (uint16_t*)&data[cnt], cfgHWordNum);
		mem_offset += sizeof(app_servo_cfg_t);
	}

	if(res)
		return !SUCCESS;
	else
		return SUCCESS;
}

static ErrorStatus app_settings_load(app_device_settings_t *data) {

	Flash_Read_HalfWordData(APP_CFG_FLASH_ADDRESS, (uint16_t*)data, (sizeof(app_device_settings_t) >> 1));

	// размер cfg без учета CRC
	uint16_t crc = Crc16((uint8_t*)&data->MB_ADDRESS, sizeof(app_device_settings_t) - sizeof(data->CRC_16));

	if(crc == data->CRC_16)
		return SUCCESS;
	else
		return !SUCCESS;
}

static ErrorStatus servo_cfg_load(app_servo_cfg_t *data, uint8_t servo_num) {

	uint16_t servosCnt = 0;
	uint32_t mem_offset = 0;
	app_servo_cfg_t *tmpCfg = data;

	// чтение памяти для поиска нужного id
	while(servosCnt < servo_num) {
		// чтение памяти настроек
		Flash_Read_HalfWordData(APP_CFG_FLASH_ADDRESS + mem_offset, (uint16_t*)tmpCfg, (sizeof(app_servo_cfg_t) >> 1));

		uint8_t crc = Crc16(&tmpCfg->servo_id, sizeof(app_servo_cfg_t) - sizeof(tmpCfg->crc_8));

		if(crc == tmpCfg->crc_8)
			tmpCfg->crc_8 = CFG_VALID;
		else
			tmpCfg->crc_8 = CFG_NOT_VALID;

		servosCnt++;
		mem_offset += sizeof(app_servo_cfg_t);
		tmpCfg = data + mem_offset;

	}
	return SUCCESS;
}

