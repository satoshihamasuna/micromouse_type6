/*
 * turn_1400.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1400_H_
#define CPP_PARAMS_TURN_1400_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_1400 = {0.4, 0.05, 0.0};
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,18.83,36.69, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,18.83,36.69,-90.0f,Turn_R};
const static t_turn_param_table slalom_L90_1400_table = {1.40f, 52.0f,15.75,30.76, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1400_table = {1.40f,-52.0f,15.75,30.76,-90.0f,Turn_R};
const static t_param param_L90_1400 = {&slalom_L90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};
const static t_param param_R90_1400 = {&slalom_R90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};

const static t_pid_gain sp_gain_turn180_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_1400 = {0.4, 0.05, 0.0};
//const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.50f,11.70,28.82, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.50f,11.70,28.82,-180.0f,Turn_R};
const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.00f,11.71,27.38, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.00f,11.71,27.38,-180.0f,Turn_R};
const static t_param param_L180_1400 = {&slalom_L180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};
const static t_param param_R180_1400 = {&slalom_R180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};

//not adjust
const static t_pid_gain sp_gain_turnV90_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_1400 = {0.3, 0.02, 0.0};
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 39.0f,4.98,30.64, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-39.0f,4.98,30.64,-90.0f,Turn_R};
const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 40.0f,5.66,20.06, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-40.0f,5.66,20.06,-90.0f,Turn_R};
const static t_param param_LV90_1400 = {&slalom_LV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};
const static t_param param_RV90_1400 = {&slalom_RV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};

const static t_pid_gain sp_gain_turnIn45_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_1400 = {0.3, 0.02, 0.0};
//const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 55.0f,7.18,42.23, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,7.18,42.23,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 55.0f,8.60,38.76, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,8.60,38.76,-45.0f,Turn_R};
const static t_param param_inL45_1400 = {&slalom_inL45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};
const static t_param param_inR45_1400 = {&slalom_inR45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_1400 = {0.2, 0.02, 0.0};
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,19.32,24.06, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,19.32,24.06,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 60.0f,22.89,17.52, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-60.0f,22.89,17.52,-45.0f,Turn_R};
const static t_param param_outL45_1400 = {&slalom_outL45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};
const static t_param param_outR45_1400 = {&slalom_outR45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};


const static t_pid_gain sp_gain_turnIn135_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1400 = {0.4, 0.02, 0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 43.0f,10.31,20.57, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-43.0f,10.31,20.57,-135.0f,Turn_R};
const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 41.0f,13.22,21.61, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-41.0f,13.22,21.61,-135.0f,Turn_R};
const static t_param param_inL135_1400 = {&slalom_inL135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};
const static t_param param_inR135_1400 = {&slalom_inR135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};

//
const static t_pid_gain sp_gain_turnOut135_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_1400 = {0.4, 0.02, 0.0};
//const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 41.0f,9.04,34.98, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-41.0f,9.04,34.98,-135.0f,Turn_R};
const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 41.0f,5.57,29.39, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-41.0f,5.57,29.39,-135.0f,Turn_R};
const static t_param param_outL135_1400 = {&slalom_outL135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};
const static t_param param_outR135_1400 = {&slalom_outR135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};

const static t_pid_gain sp_gain_long_turnV90_1400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_long_turnV90_1400 = {0.4, 0.01, 0.0};
//const static t_turn_param_table slalom_LongLV90_1400_table = {1.40f, 76.0f,17.59,42.84, 90.0f,Turn_L};
//const static t_turn_param_table slalom_LongRV90_1400_table = {1.40f,-76.0f,17.59,42.84,-90.0f,Turn_R};
const static t_turn_param_table slalom_LongLV90_1400_table = {1.40f, 76.0f,19.99,32.66, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_1400_table = {1.40f,-76.0f,19.99,32.66,-90.0f,Turn_R};
const static t_param param_LongLV90_1400 = {&slalom_LongLV90_1400_table,&sp_gain_long_turnV90_1400,&om_gain_long_turnV90_1400};
const static t_param param_LongRV90_1400 = {&slalom_LongRV90_1400_table,&sp_gain_long_turnV90_1400,&om_gain_long_turnV90_1400};


const static t_param *const mode_1400[TURN_MODES] = 	{	&param_dummy,			&param_dummy,		&param_dummy,
															&param_R90_1400,		&param_L90_1400,
															&param_R180_1400,		&param_L180_1400,
															&param_inR45_1400,		&param_inL45_1400,
															&param_outR45_1400,		&param_outL45_1400,
															&param_inR135_1400,		&param_inL135_1400,
															&param_outR135_1400,	&param_outL135_1400,
															&param_RV90_1400,		&param_LV90_1400,
															NULL,		NULL
														};


const static t_param *const mode_1400_v2[] = 	{			NULL,			NULL,		NULL,
															&param_R90_1400,		&param_L90_1400,
															&param_R180_1400,		&param_L180_1400,
															NULL,					NULL,
															NULL,					NULL,
															&param_inR135_1400,		&param_inL135_1400,
															&param_outR135_1400,	&param_outL135_1400,
															NULL,		NULL,
															NULL,		NULL
														};

const static t_param *const mode_1400_v3[TURN_MODES] = 	{	NULL,			NULL,		NULL,
															&param_R90_1400,		&param_L90_1400,
															&param_R180_1400,		&param_L180_1400,
															&param_inR45_1400,		&param_inL45_1400,
															&param_outR45_1400,		&param_outL45_1400,
															&param_inR135_1400,		&param_inL135_1400,
															&param_outR135_1400,	&param_outL135_1400,
															&param_RV90_1400,		&param_LV90_1400,
															NULL,		NULL,
														};




#endif /* CPP_PARAMS_TURN_1400_H_ */
