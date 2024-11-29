/*
 * turn_1600.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_1600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_1600 = {0.5, 0.05, 0.0};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,13.46,32.80, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,13.46,32.80,-90.0f,Turn_R};
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.44,33.55, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.44,33.55,-90.0f,Turn_R};
const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};

const static t_pid_gain sp_gain_turn180_1600 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_turn180_1600 = {0.4, 0.05, 0.0};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 46.0f,12.93,35.11, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-46.0f,12.93,35.11,-180.0f,Turn_R};
//k = 250
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,10.58,31.60, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,10.58,31.60,-180.0f,Turn_R};
const static t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};
const static t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};

/*
const static t_pid_gain sp_gain_turnIn135_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnIn135_1400 = {0.4, 0.02, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 43.0f,10.31,20.57, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-43.0f,10.31,20.57,-135.0f,Turn_R};
const static t_param param_inL135_1400 = {&slalom_inL135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};
const static t_param param_inR135_1400 = {&slalom_inR135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};

//
const static t_pid_gain sp_gain_turnOut135_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnOut135_1400 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 41.0f,9.04,34.98, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-41.0f,9.04,34.98,-135.0f,Turn_R};
const static t_param param_outL135_1400 = {&slalom_outL135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};
const static t_param param_outR135_1400 = {&slalom_outR135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};
*/
const static t_pid_gain sp_gain_long_turnV90_1600 = {6.5, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_1600 = {0.4, 0.01, 0.0};
//const static t_turn_param_table slalom_LongLV90_1600_table = {1.60f, 76.0f,17.88,36.96, 90.0f,Turn_L};
//const static t_turn_param_table slalom_LongRV90_1600_table = {1.60f,-76.0f,17.88,36.96,-90.0f,Turn_R};
const static t_turn_param_table slalom_LongLV90_1600_table = {1.60f, 76.0f,17.13,39.64, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_1600_table = {1.60f,-76.0f,17.13,39.64,-90.0f,Turn_R};

const static t_param param_LongLV90_1600 = {&slalom_LongLV90_1600_table,&sp_gain_long_turnV90_1600,&om_gain_long_turnV90_1600};
const static t_param param_LongRV90_1600 = {&slalom_LongRV90_1600_table,&sp_gain_long_turnV90_1600,&om_gain_long_turnV90_1600};


const static t_param *const mode_1600[TURN_MODES] = 	{	NULL,					NULL,			NULL,
															&param_R90_1600,		&param_L90_1600,
															&param_R180_1600,		&param_L180_1600,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
														};





#endif /* CPP_PARAMS_TURN_1600_H_ */
