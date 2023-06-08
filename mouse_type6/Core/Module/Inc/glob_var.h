/*
 * glob_var.h
 *
 *  Created on: Jun 7, 2023
 *      Author: sato1
 */

#include "typedef.h"
#include "macro.h"

#ifndef MODULE_INC_GLOB_VAR_H_
#define MODULE_INC_GLOB_VAR_H_
#define GLOBAL
#else
#define GLOBAL extern
#endif

GLOBAL t_encoder enc_R,enc_L;
GLOBAL uint8_t mouse_mode;
GLOBAL uint8_t is_mode_enable;

GLOBAL uint8_t run_mode;
GLOBAL t_sp_param machine;
GLOBAL t_sp_param target;
GLOBAL t_sp_param max_set;

/* MODULE_INC_GLOB_VAR_H_ */
