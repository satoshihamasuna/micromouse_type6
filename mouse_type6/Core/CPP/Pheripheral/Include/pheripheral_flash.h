/*
 * pheripheral_flash.h
 *
 *  Created on: 2024/08/10
 *      Author: sato1
 */

#ifndef CPP_PHERIPHERAL_PHERIPHERAL_FLASH_H_
#define CPP_PHERIPHERAL_PHERIPHERAL_FLASH_H_

#include "stm32f4xx_hal.h"

#include "index.h"
#include "macro.h"
#include "../../Subsys/Inc/wall_class.h"

#include <string.h>
#include <stdint.h>

#define START_ADDRESS  	0x08004000
#define END_ADDRESS    	0x08007FFF

/*
#define MAP_START_ADDRESS 	0x8060000
#define MAP_END_ADDRESS	  	0x806FFFF

#define WALL_START_ADDRESS  0x8070000
#define WALL_END_ADDRESS	0x807FFFF
*/
#define FLASH_SECTOR_SIZE    1024*16

void eraseALL();
void eraseFlash( void );
void writeFlash(uint32_t address, uint16_t *data );
void loadFlash(uint32_t address,uint16_t *data, uint32_t size );

uint8_t* Flash_Load( void );
uint8_t Flash_Save( void );

//static uint8_t _flash_start;

void work_ram_set(uint32_t position,uint8_t data);

uint8_t work_ram_read(uint32_t position);

#endif /* CPP_PHERIPHERAL_PHERIPHERAL_FLASH_H_ */
