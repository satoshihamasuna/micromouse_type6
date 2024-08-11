/*
 * flash.cpp
 *
 *  Created on: 2024/03/07
 *      Author: sato1
 */

#include "../Inc/flash.h"
#include "../../Subsys/Inc/wall_class.h"
#include "../../Pheripheral/Include/typedef.h"
#include "../../Pheripheral/Include/pheripheral_flash.h"


void read_wall_flash(wall_class *wall_property){
	Flash_Load();
	for(uint32_t x = 0 ; x < MAZE_SIZE_X ; x++ ){
		for(uint32_t y = 0 ; y < MAZE_SIZE_Y ; y++ ){
			uint8_t data = work_ram_read((x << 6 | y));
			wall_property->wall[x][y].north = (data >> 6) & 0x0003;
			wall_property->wall[x][y].east  = (data >> 4) & 0x0003;
			wall_property->wall[x][y].south = (data >> 2) & 0x0003;
			wall_property->wall[x][y].west  = (data >> 0) & 0x0003;
		}
	}
}

void write_wall_flash(wall_class *wall_property){
	for(uint32_t x = 0 ; x < MAZE_SIZE_X ; x++ ){
		for(uint32_t y = 0 ; y < MAZE_SIZE_Y ; y++ ){
			uint8_t data = (((uint8_t)(wall_property->wall[x][y].north)<< 6) | ((uint8_t)(wall_property->wall[x][y].east) << 4)
					  | ((uint8_t)(wall_property->wall[x][y].south) << 2) | ((uint8_t)(wall_property->wall[x][y].west) << 0));
			work_ram_set((x << 6 | y),data);
		}
	}
	Flash_Save();
}


void write_save_data(wall_class *wall_property){
	write_wall_flash(&(*wall_property));
}

void read_save_data(wall_class *wall_property){
	read_wall_flash(&(*wall_property));
}
