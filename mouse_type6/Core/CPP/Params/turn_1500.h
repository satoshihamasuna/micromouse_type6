/*
 * turn_1500.h
 *
 *  Created on: 2024/11/29
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1500_H_
#define CPP_PARAMS_TURN_1500_H_




















const static t_pid_gain sp_gain_turn90_1500 = {7.50, 0.01, 0.0};
const static t_pid_gain om_gain_turn90_1500 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_L90_1500_table = {1.50f, 50.0f,17.02,33.86, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1500_table = {1.50f,-50.0f,17.02,33.86,-90.0f,Turn_R};
const static t_param param_L90_1500 = {&slalom_L90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};
const static t_param param_R90_1500 = {&slalom_R90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};

const static t_pid_gain sp_gain_turn180_1500 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_turn180_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1500_table = {1.50f, 46.0f,13.69,33.69, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1500_table = {1.50f,-46.0f,13.69,33.69,-180.0f,Turn_R};
const static t_param param_L180_1500 = {&slalom_L180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};
const static t_param param_R180_1500 = {&slalom_R180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};

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

const static t_pid_gain sp_gain_long_turnV90_1400 = {6.5, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_1400 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_LongLV90_1400_table = {1.40f, 76.0f,17.59,42.84, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_1400_table = {1.40f,-76.0f,17.59,42.84,-90.0f,Turn_R};
const static t_param param_LongLV90_1400 = {&slalom_LongLV90_1400_table,&sp_gain_long_turnV90_1400,&om_gain_long_turnV90_1400};
const static t_param param_LongRV90_1400 = {&slalom_LongRV90_1400_table,&sp_gain_long_turnV90_1400,&om_gain_long_turnV90_1400};
*/

const static t_param *const mode_1500[TURN_MODES] = 	{	NULL,					NULL,			NULL,
															&param_R90_1500,		&param_L90_1500,
															&param_R180_1500,		&param_L180_1500,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
															NULL,	NULL,
														};


#endif /* CPP_PARAMS_TURN_1500_H_ */
