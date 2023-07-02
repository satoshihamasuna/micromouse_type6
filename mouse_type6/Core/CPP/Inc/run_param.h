/*
 * run_pram.h
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_PARAM_H_
#define CPP_INC_RUN_PARAM_H_


#include "run_task.h"
#include "typedef.h"

const static t_pid_gain sp_gain_search_turn = {6.0, 0.05, 0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.4, 0.005, 0.0};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_table = {0.30f, 25.0f,14.72-3.0,11.93+0.0, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table = {0.30f,-25.0f,14.72-3.0,11.93+0.0,-90.0f,Turn_R};
const static t_param param_L90_search = {&slalom_L90_table ,&sp_gain_search_turn,&om_gain_search_turn};
const static t_param param_R90_search = {&slalom_R90_table, &sp_gain_search_turn,&om_gain_search_turn};


#endif /* CPP_INC_RUN_PARAM_H_ */
