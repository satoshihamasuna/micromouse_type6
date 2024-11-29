/*
 * turn_500.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "typedef_run_param.h"

//-----------velo = 500 mm/s parameters
const static t_pid_gain sp_gain_turn90_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 50.0f,20.76,26.77, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_500_table = {0.50f,-50.0f,20.76,26.77,-90.0f,Turn_R};// k= 100
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,22.4,25.48, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,22.4,25.48,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 40.5f,7.40,13.74, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-40.5f,7.40,13.74,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 55.0f,10.47,35.33, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-55.0f,10.47,35.33,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.4, 0.05, 0.0};;
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 55.0f,29.15,16.66, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-55.0f,29.15,16.66,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 40.0f,14.04,12.61, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-40.0f,14.04,12.61,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 37.0f,16.28,31.09, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-37.0f,16.28,31.09,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};

const static t_pid_gain sp_gain_long_turnV90_500 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_500 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_LongLV90_500_table = {0.50f, 70.0f,27.82,46.56, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_500_table = {0.50f,-70.0f,27.82,46.56,-90.0f,Turn_R};
const static t_param param_LongLV90_500 = {&slalom_LongLV90_500_table,&sp_gain_long_turnV90_500,&om_gain_long_turnV90_500};
const static t_param param_LongRV90_500 = {&slalom_LongRV90_500_table,&sp_gain_long_turnV90_500,&om_gain_long_turnV90_500};

const static t_param *const mode_500[TURN_MODES] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
														&param_R90_500,		&param_L90_500,
														&param_R180_500,	&param_L180_500,
														&param_inR45_500,	&param_inL45_500,
														&param_outR45_500,	&param_outL45_500,
														&param_inR135_500,	&param_inL135_500,
														&param_outR135_500,	&param_outL135_500,
														&param_RV90_500,	&param_LV90_500,
														NULL,	NULL
													};

const static t_param *const mode_500_v2[TURN_MODES] = 	{	NULL,	NULL,		NULL,
															&param_R90_500,		&param_L90_500,
															&param_R180_500,	&param_L180_500,
															&param_inR45_500,	&param_inL45_500,
															&param_outR45_500,	&param_outL45_500,
															&param_inR135_500,	&param_inL135_500,
															&param_outR135_500,	&param_outL135_500,
															&param_RV90_500,	&param_LV90_500,
															NULL,	NULL
														};


#endif /* CPP_PARAMS_TURN_500_H_ */
