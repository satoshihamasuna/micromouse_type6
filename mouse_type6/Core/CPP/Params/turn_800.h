/*
 * turn_800.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_800_H_
#define CPP_PARAMS_TURN_800_H_

#include "typedef_run_param.h"

//-----------velo = 800 mm/s parameters
const static t_pid_gain sp_gain_turn90_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_800 = {0.2, 0.01, 0.0};
const static t_turn_param_table slalom_L90_800_table = {0.80f, 55.0f,11.66,32.53, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_800_table = {0.80f,-55.0f,11.66,32.53,-90.0f,Turn_R};// k= 100
const static t_param param_L90_800 = {&slalom_L90_800_table,&sp_gain_turn90_800,&om_gain_turn90_800};
const static t_param param_R90_800 = {&slalom_R90_800_table,&sp_gain_turn90_800,&om_gain_turn90_800};

const static t_pid_gain sp_gain_turn180_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_800 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_L180_800_table = {0.80f, 50.0f,8.54,29.60, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_800_table = {0.80f,-50.0f,8.54,29.60,-180.0f,Turn_R};
const static t_param param_L180_800 = {&slalom_L180_800_table,&sp_gain_turn180_800,&om_gain_turn180_800};
const static t_param param_R180_800 = {&slalom_R180_800_table,&sp_gain_turn180_800,&om_gain_turn180_800};

const static t_pid_gain sp_gain_turnV90_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_800 = {0.2, 0.005, 0.0};
const static t_turn_param_table slalom_LV90_800_table = {0.80f, 40.0f,4.86,21.94, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_800_table = {0.80f,-40.0f,4.86,21.94,-90.0f,Turn_R};
const static t_param param_LV90_800 = {&slalom_LV90_800_table,&sp_gain_turnV90_800,&om_gain_turnV90_800};
const static t_param param_RV90_800 = {&slalom_RV90_800_table,&sp_gain_turnV90_800,&om_gain_turnV90_800};

const static t_pid_gain sp_gain_turnIn45_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_800 = {0.2, 0.00, 0.0};
const static t_turn_param_table slalom_inL45_800_table = {0.80f, 60.0f,4.28,37.08, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_800_table = {0.80f,-60.0f,4.28,37.08,-45.0f,Turn_R};
const static t_param param_inL45_800 = {&slalom_inL45_800_table,&sp_gain_turnIn45_800,&om_gain_turnIn45_800};
const static t_param param_inR45_800 = {&slalom_inR45_800_table,&sp_gain_turnIn45_800,&om_gain_turnIn45_800};

const static t_pid_gain sp_gain_turnOut45_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_800 = {0.2, 0.00, 0.0};;
const static t_turn_param_table slalom_outL45_800_table = {0.80f, 75.0f,12.97,9.99, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_800_table = {0.80f,-75.0f,12.97,9.99,-45.0f,Turn_R};
const static t_param param_outL45_800 = {&slalom_outL45_800_table,&sp_gain_turnOut45_800,&om_gain_turnOut45_800};
const static t_param param_outR45_800 = {&slalom_outR45_800_table,&sp_gain_turnOut45_800,&om_gain_turnOut45_800};

const static t_pid_gain sp_gain_turnIn135_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_800 = {0.4, 0.005, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_800_table = {0.80f, 40.0f,22.93,37.86, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_800_table = {0.80f,-40.0f,22.93,37.86,-135.0f,Turn_R};
const static t_param param_inL135_800 = {&slalom_inL135_800_table,&sp_gain_turnIn135_800,&om_gain_turnIn135_800};
const static t_param param_inR135_800 = {&slalom_inR135_800_table,&sp_gain_turnIn135_800,&om_gain_turnIn135_800};

const static t_pid_gain sp_gain_turnOut135_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_800 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_outL135_800_table = {0.80f, 40.0f,15.09,44.05, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_800_table = {0.80f,-40.0f,15.09,44.05,-135.0f,Turn_R};
const static t_param param_outL135_800 = {&slalom_outL135_800_table,&sp_gain_turnOut135_800,&om_gain_turnOut135_800};
const static t_param param_outR135_800 = {&slalom_outR135_800_table,&sp_gain_turnOut135_800,&om_gain_turnOut135_800};

const static t_pid_gain sp_gain_long_turnV90_800 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_800 = {0.2, 0.005, 0.0};
const static t_turn_param_table slalom_LongLV90_800_table = {0.80f, 75.0f,19.09,35.26, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_800_table = {0.80f,-75.0f,19.09,35.26,-90.0f,Turn_R};
const static t_param param_LongLV90_800 = {&slalom_LongLV90_800_table,&sp_gain_long_turnV90_800,&om_gain_long_turnV90_800};
const static t_param param_LongRV90_800 = {&slalom_LongRV90_800_table,&sp_gain_long_turnV90_800,&om_gain_long_turnV90_800};

const static t_param *const mode_800[TURN_MODES] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
														&param_R90_800,		&param_L90_800,
														&param_R180_800,	&param_L180_800,
														&param_inR45_800,	&param_inL45_800,
														&param_outR45_800,	&param_outL45_800,
														&param_inR135_800,	&param_inL135_800,
														&param_outR135_800,	&param_outL135_800,
														&param_RV90_800,	&param_LV90_800,
														&param_LongRV90_800,	&param_LongLV90_800
													};


#endif /* CPP_PARAMS_TURN_800_H_ */
