/*
 * turn_650.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_650_H_
#define CPP_PARAMS_TURN_650_H_

#include "typedef_run_param.h"

//-----------velo = 650 mm/s parameters
const static t_pid_gain sp_gain_turn90_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_650 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_650_table = {0.65f, 55.0f,13.37,27.82, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_650_table = {0.65f,-55.0f,13.37,27.82,-90.0f,Turn_R};// k= 100
const static t_param param_L90_650 = {&slalom_L90_650_table,&sp_gain_turn90_650,&om_gain_turn90_650};
const static t_param param_R90_650 = {&slalom_R90_650_table,&sp_gain_turn90_650,&om_gain_turn90_650};

const static t_pid_gain sp_gain_turn180_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_650 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_650_table = {0.65f, 45.0f,17.46,28.59, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_650_table = {0.65f,-45.0f,17.46,28.59,-180.0f,Turn_R};
const static t_param param_L180_650 = {&slalom_L180_650_table,&sp_gain_turn180_650,&om_gain_turn180_650};
const static t_param param_R180_650 = {&slalom_R180_650_table,&sp_gain_turn180_650,&om_gain_turn180_650};

const static t_pid_gain sp_gain_turnV90_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_650 = {0.2, 0.005, 0.0};
const static t_turn_param_table slalom_LV90_650_table = {0.65f, 40.5f,6.60,17.91, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_650_table = {0.65f,-40.5f,6.60,17.91,-90.0f,Turn_R};
const static t_param param_LV90_650 = {&slalom_LV90_650_table,&sp_gain_turnV90_650,&om_gain_turnV90_650};
const static t_param param_RV90_650 = {&slalom_RV90_650_table,&sp_gain_turnV90_650,&om_gain_turnV90_650};

const static t_pid_gain sp_gain_turnIn45_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_650 = {0.2, 0.00, 0.0};
const static t_turn_param_table slalom_inL45_650_table = {0.65f, 62.0f,4.68,33.35, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_650_table = {0.65f,-62.0f,4.68,33.35,-45.0f,Turn_R};
const static t_param param_inL45_650 = {&slalom_inL45_650_table,&sp_gain_turnIn45_650,&om_gain_turnIn45_650};
const static t_param param_inR45_650 = {&slalom_inR45_650_table,&sp_gain_turnIn45_650,&om_gain_turnIn45_650};

const static t_pid_gain sp_gain_turnOut45_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_650 = {0.2, 0.00, 0.0};;
const static t_turn_param_table slalom_outL45_650_table = {0.65f, 72.0f,17.17,8.75, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_650_table = {0.65f,-72.0f,17.17,8.75,-45.0f,Turn_R};
const static t_param param_outL45_650 = {&slalom_outL45_650_table,&sp_gain_turnOut45_650,&om_gain_turnOut45_650};
const static t_param param_outR45_650 = {&slalom_outR45_650_table,&sp_gain_turnOut45_650,&om_gain_turnOut45_650};

const static t_pid_gain sp_gain_turnIn135_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_650 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_650_table = {0.65f, 40.0f,19.60,25.46, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_650_table = {0.65f,-40.0f,19.60,25.46,-135.0f,Turn_R};
const static t_param param_inL135_650 = {&slalom_inL135_650_table,&sp_gain_turnIn135_650,&om_gain_turnIn135_650};
const static t_param param_inR135_650 = {&slalom_inR135_650_table,&sp_gain_turnIn135_650,&om_gain_turnIn135_650};

const static t_pid_gain sp_gain_turnOut135_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_650 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_650_table = {0.65f, 37.0f,21.38,42.76, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_650_table = {0.65f,-37.0f,21.38,42.76,-135.0f,Turn_R};
const static t_param param_outL135_650 = {&slalom_outL135_650_table,&sp_gain_turnOut135_650,&om_gain_turnOut135_650};
const static t_param param_outR135_650 = {&slalom_outR135_650_table,&sp_gain_turnOut135_650,&om_gain_turnOut135_650};

const static t_pid_gain sp_gain_long_turnV90_650 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_650 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_LongLV90_650_table = {0.65f, 80.0f,14.96,25.33, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_650_table = {0.65f,-80.0f,14.96,25.33,-90.0f,Turn_R};
const static t_param param_LongLV90_650 = {&slalom_LongLV90_650_table,&sp_gain_long_turnV90_650,&om_gain_long_turnV90_650};
const static t_param param_LongRV90_650 = {&slalom_LongRV90_650_table,&sp_gain_long_turnV90_650,&om_gain_long_turnV90_650};

const static t_param *const mode_650[TURN_MODES] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
														&param_R90_650,		&param_L90_650,
														&param_R180_650,	&param_L180_650,
														&param_inR45_650,	&param_inL45_650,
														&param_outR45_650,	&param_outL45_650,
														&param_inR135_650,	&param_inL135_650,
														&param_outR135_650,	&param_outL135_650,
														&param_RV90_650,	&param_LV90_650,
														&param_LongRV90_650,	&param_LongLV90_650
													};

const static t_param *const mode_650_v2[TURN_MODES] = 	{	NULL,		NULL,		NULL,
															&param_R90_650,		&param_L90_650,
															&param_R180_650,	&param_L180_650,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															&param_LongRV90_650,	&param_LongLV90_650
														};



#endif /* CPP_PARAMS_TURN_650_H_ */
