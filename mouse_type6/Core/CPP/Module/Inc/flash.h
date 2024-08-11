/*
 * flash.h
 *
 *  Created on: 2023/07/03
 *      Author: sato1
 */

#ifndef CPP_INC_FLASH_H_
#define CPP_INC_FLASH_H_

#include "../../Subsys/Inc/wall_class.h"
#include "../../Pheripheral/Include/typedef.h"
#include "../../Pheripheral/Include/pheripheral_flash.h"

void write_save_data(wall_class *wall_property);
//ここをポインタのポインタに変更する
void read_save_data(wall_class *wall_property);
//ここをポインタのポインタに変更する

#endif /* CPP_INC_FLASH_H_ */
