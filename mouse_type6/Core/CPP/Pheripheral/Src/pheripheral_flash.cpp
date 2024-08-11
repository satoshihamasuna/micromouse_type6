/*
 * pheripheral_flash.cpp
 *
 *  Created on: 2024/08/10
 *      Author: sato1
 */

#include "stm32f4xx_hal.h"

#include "../Include/index.h"
#include "../Include/macro.h"
#include "../Include/pheripheral_flash.h"

#include <string.h>
#include <stdint.h>

uint8_t store_ram[FLASH_SECTOR_SIZE];

void eraseALL(){
	HAL_FLASH_Unlock();
	eraseFlash();
	HAL_FLASH_Lock();
}


void eraseFlash( void )
{
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// select sector
	erase.Sector = FLASH_SECTOR_1;		       // set selector1
	erase.NbSectors = 1;		// set to erase one sector
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
}

/*
 * @brief write flash(sector1)
 * @param uint32_t address sector1 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint16_t *data )
{
	//HAL_FLASH_Unlock();		// unlock flash

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, *data); // write byte

	//HAL_FLASH_Lock();		// lock flash
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
*/
void loadFlash(uint32_t address,uint16_t *data, uint32_t size )
{
	memcpy(data, (uint16_t*) address, size); // copy data
}


uint8_t* Flash_Load( void )
{
	uint32_t address = START_ADDRESS;
	for(uint32_t i = 0; i <  FLASH_SECTOR_SIZE;i++)
	{
		uint8_t data;
		memcpy(&data, (uint8_t*) (address + i) , 1);
		store_ram[i] = data;
	}
	return store_ram;
}

/* ----------------------------------------------------------------------------------
	Flashのsector1を消去後、store_ramにあるデータを書き込む
-----------------------------------------------------------------------------------*/
uint8_t Flash_Save( void )
{
	// Flashをclear
	//if( !Flash_Clear() ) return 0;

	HAL_FLASH_Unlock();
	eraseFlash();

	uint32_t *p_work_ram = (uint32_t*)store_ram;

	//HAL_FLASH_Unlock();

	// work_ramにあるデータを4バイトごとまとめて書き込む
	HAL_StatusTypeDef result;
	const size_t write_cnt = FLASH_SECTOR_SIZE / sizeof(uint32_t);

	for( size_t i=0; i<write_cnt; i++ ) {
		result = HAL_FLASH_Program(
			FLASH_TYPEPROGRAM_WORD,
			(uint32_t)(START_ADDRESS) + sizeof(uint32_t) * i,
			p_work_ram[i]
		);

		if( result != HAL_OK ) break;
	}

	HAL_FLASH_Lock();

	return result == HAL_OK;
}

void work_ram_set(uint32_t position,uint8_t data)
{
	store_ram[position] = data;
}

uint8_t work_ram_read(uint32_t position)
{
	return store_ram[position];
}


