/*
 * turn_1200.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1200_H_
#define CPP_PARAMS_TURN_1200_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_1200 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_turn90_1200 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_L90_1200_table = {1.20f, 46.5f,24.56,37.65, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1200_table = {1.20f,-46.5f,24.56,37.65,-90.0f,Turn_R};
const static t_param param_L90_1200 = {&slalom_L90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200};
const static t_param param_R90_1200 = {&slalom_R90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200};

const static t_pid_gain sp_gain_turn180_1200 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_turn180_1200 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1200_table = {1.20f, 46.5f,15.21,29.73, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1200_table = {1.20f,-46.5f,15.21,29.73,-180.0f,Turn_R};
const static t_param param_L180_1200 = {&slalom_L180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200};
const static t_param param_R180_1200 = {&slalom_R180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200};

//not adjust
const static t_pid_gain sp_gain_turnV90_1200 = {6.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnV90_1200 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_LV90_1200_table = {1.20f, 38.5f,7.63,27.52, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1200_table = {1.20f,-38.5f,7.63,27.52,-90.0f,Turn_R};
const static t_param param_LV90_1200 = {&slalom_LV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};
const static t_param param_RV90_1200 = {&slalom_RV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};

const static t_pid_gain sp_gain_turnIn45_1200 = {6.5, 0.02, 0.0};
const static t_pid_gain om_gain_turnIn45_1200 = {0.2, 0.005, 0.0};
const static t_turn_param_table slalom_inL45_1200_table = {1.20f, 52.0f,9.40,41.43, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1200_table = {1.20f,-52.0f,9.40,41.43,-45.0f,Turn_R};
const static t_param param_inL45_1200 = {&slalom_inL45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};
const static t_param param_inR45_1200 = {&slalom_inR45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1200 = {6.5, 0.02, 0.0};
const static t_pid_gain om_gain_turnOut45_1200 = {0.2, 0.005, 0.0};
const static t_turn_param_table slalom_outL45_1200_table = {1.20f, 52.0f,24.40,22.46, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1200_table = {1.20f,-52.0f,24.40,22.46,-45.0f,Turn_R};
const static t_param param_outL45_1200 = {&slalom_outL45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200};
const static t_param param_outR45_1200 = {&slalom_outR45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200};


const static t_pid_gain sp_gain_turnIn135_1200 = {6.5, 0.02, 0.0};
const static t_pid_gain om_gain_turnIn135_1200 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1200_table = {1.20f, 40.0f,21.73,28.87, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1200_table = {1.20f,-40.0f,21.73,28.87,-135.0f,Turn_R};
const static t_param param_inL135_1200 = {&slalom_inL135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200};
const static t_param param_inR135_1200 = {&slalom_inR135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200};

//
const static t_pid_gain sp_gain_turnOut135_1200 = {6.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnOut135_1200 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1200_table = {1.20f, 41.0f,5.96,26.60, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1200_table = {1.20f,-41.0f,5.96,26.60,-135.0f,Turn_R};
const static t_param param_outL135_1200 = {&slalom_outL135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200};
const static t_param param_outR135_1200 = {&slalom_outR135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200};

const static t_pid_gain sp_gain_long_turnV90_1200 = {6.5, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_1200 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_LongLV90_1200_table = {1.20f, 70.0f,27.82,46.56, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_1200_table = {1.20f,-70.0f,27.82,46.56,-90.0f,Turn_R};
const static t_param param_LongLV90_1200 = {&slalom_LongLV90_1200_table,&sp_gain_long_turnV90_1200,&om_gain_long_turnV90_1200};
const static t_param param_LongRV90_1200 = {&slalom_LongRV90_1200_table,&sp_gain_long_turnV90_1200,&om_gain_long_turnV90_1200};



const static t_param *const mode_1200[TURN_MODES] = 	{	&param_dummy,			&param_dummy,		&param_dummy,
															&param_R90_1200,		&param_L90_1200,
															&param_R180_1200,		&param_L180_1200,
															&param_inR45_1200,		&param_inL45_1200,
															&param_outR45_1200,		&param_outL45_1200,
															&param_inR135_1200,		&param_inL135_1200,
															&param_outR135_1200,	&param_outL135_1200,
															&param_RV90_1200,		&param_LV90_1200,
															NULL,		NULL
														};

const static t_param *const mode_1200_v2[] = 	{	NULL,			NULL,		NULL,
															&param_R90_1200,		&param_L90_1200,
															&param_R180_1200,		&param_L180_1200,
															&param_inR45_1200,		&param_inL45_1200,
															&param_outR45_1200,		&param_outL45_1200,
															&param_inR135_1200,		&param_inL135_1200,
															&param_outR135_1200,	&param_outL135_1200,
															&param_RV90_1200,		&param_LV90_1200,
															NULL,	NULL
														};



#endif /* CPP_PARAMS_TURN_1200_H_ */
