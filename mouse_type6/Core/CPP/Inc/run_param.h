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


const static t_pid_gain sp_gain_dummy = {0.0f,0.0f,0.0f};
const static t_pid_gain om_gain_dummy = {0.0f, 0.0f, 0.0f};
const static t_turn_param_table slalom_dummy = {0.0f,0.0f,0.0f,0.0f,0.0f,Turn_L};
const static t_param param_dummy = {&slalom_dummy,&sp_gain_dummy,&om_gain_dummy};

const static t_pid_gain sp_gain_turn90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_L90_300_table = {0.30f, 37.5f,39.05,39.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_300_table = {0.30f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_300 = {&slalom_L90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};
const static t_param param_R90_300 = {&slalom_R90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};

const static t_pid_gain sp_gain_turn180_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_L180_300_table = {0.30f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_300_table = {0.30f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_300 = {&slalom_L180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};
const static t_param param_R180_300 = {&slalom_R180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};

const static t_pid_gain sp_gain_turnV90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_LV90_300_table = {0.30f, 30.0f,22.78,23.50, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_300_table = {0.30f,-30.0f,22.78,23.50,-90.0f,Turn_R};
const static t_param param_LV90_300 = {&slalom_LV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};
const static t_param param_RV90_300 = {&slalom_RV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};

const static t_pid_gain sp_gain_turnIn45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_inL45_300_table = {0.30f, 30.0f,27.04,46.34, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_300_table = {0.30f,-30.0f,27.04,46.34,-45.0f,Turn_R};
const static t_param param_inL45_300 = {&slalom_inL45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};
const static t_param param_inR45_300 = {&slalom_inR45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};

const static t_pid_gain sp_gain_turnOut45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_outL45_300_table = {0.30f, 30.0f,45.68,27.70, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_300_table = {0.30f,-30.0f,45.68,27.70,-45.0f,Turn_R};
const static t_param param_outL45_300 = {&slalom_outL45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};
const static t_param param_outR45_300 = {&slalom_outR45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};

const static t_pid_gain sp_gain_turnIn135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_inL135_300_table = {0.30f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_300_table = {0.30f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_300 = {&slalom_inL135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};
const static t_param param_inR135_300 = {&slalom_inR135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};

const static t_pid_gain sp_gain_turnOut135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_outL135_300_table = {0.30f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_300_table = {0.30f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_300 = {&slalom_outL135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};
const static t_param param_outR135_300 = {&slalom_outR135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};

const static t_param *const mode_300[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_300,		&param_L90_300,
												&param_R180_300,	&param_L180_300,
												&param_inR45_300,	&param_inL45_300,
												&param_outR45_300,	&param_outL45_300,
												&param_inR135_300,	&param_inL135_300,
												&param_outR135_300,	&param_outL135_300,
												&param_RV90_300,	&param_LV90_300
											};


#endif /* CPP_INC_RUN_PARAM_H_ */
