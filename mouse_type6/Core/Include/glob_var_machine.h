/*
 * glob_var_machine.h
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */

#ifndef MODULE_INC_GLOB_VAR_MACHINE_H_
#define MODULE_INC_GLOB_VAR_MACHINE_H_

#include "typedef.h"
#include "macro.h"

#define GLOBAL
#else
#define GLOBAL extern
#endif

GLOBAL uint16_t mouse_mode;
GLOBAL uint16_t is_mode_enable;
GLOBAL uint16_t run_mode;
GLOBAL t_sensor sen_fr,sen_fl,sen_r,sen_l;
GLOBAL t_encoder enc_R,enc_L;

//#endif /* MODULE_INC_GLOB_VAR_MACHINE_H_ */
