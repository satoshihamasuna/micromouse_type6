/*
 * run_pram.h
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_PARAM_H_
#define CPP_INC_RUN_PARAM_H_


#include "run_task.h"
#include "../../Module/Include/typedef.h"

const static t_pid_gain sp_gain_search_turn = {6.0, 0.05, 0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.4, 0.01, 0.0};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_table = {0.30f, 26.50f,8.50,10.71, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table = {0.30f,-26.50f,8.50,10.71,-90.0f,Turn_R};
const static t_param param_L90_search = {&slalom_L90_table ,&sp_gain_search_turn,&om_gain_search_turn};
const static t_param param_R90_search = {&slalom_R90_table, &sp_gain_search_turn,&om_gain_search_turn};

const static t_pid_gain sp_gain_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_300 = {0.05, 0.01, 0.00};
const static t_velo_param param_300 = {0.30f,4.0f};
const static t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300};

const static t_pid_gain sp_gain_450 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_450 = {0.05, 0.01, 0.00};
const static t_velo_param param_450 = {0.45f,6.0f};
const static t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450};

const static t_pid_gain sp_gain_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_500 = {0.05, 0.01, 0.00};
const static t_velo_param param_500 = {0.50f,6.0f};
const static t_straight_param st_param_500 = {&param_500,&sp_gain_500,&om_gain_500};

const static t_pid_gain sp_gain_600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_600 = {0.05, 0.01, 0.00};
const static t_velo_param param_600 = {0.60f,6.0f};
const static t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600};

const static t_pid_gain sp_gain_700 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_700 = {0.05, 0.01, 0.00};
const static t_velo_param param_700 = {0.70f,6.0f};
const static t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700};

const static t_pid_gain sp_gain_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1000 = {0.05, 0.01, 0.00};
const static t_velo_param param_1000 = {1.0f,9.0f};
const static t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};

const static t_pid_gain sp_gain_1050 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1050 = {0.05, 0.01, 0.00};
const static t_velo_param param_1050 = {1.05f,9.0f};
const static t_straight_param st_param_1050 = {&param_1050,&sp_gain_1050,&om_gain_1050};

const static t_pid_gain sp_gain_1100 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1100 = {0.05, 0.01, 0.00};
const static t_velo_param param_1100 = {1.10f,9.0f};
const static t_straight_param st_param_1100 = {&param_1100,&sp_gain_1100,&om_gain_1100};

const static t_pid_gain sp_gain_1200 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1200 = {0.05, 0.01, 0.00};
const static t_velo_param param_1200 = {1.20f,10.0f};
const static t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};

const static t_pid_gain sp_gain_1300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1300 = {0.05, 0.01, 0.00};
const static t_velo_param param_1300 = {1.30f,10.0f};
const static t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};

const static t_pid_gain sp_gain_1400 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_1400 = {0.05, 0.01, 0.00};
const static t_velo_param param_1400 = {1.40f,12.0f};
const static t_straight_param st_param_1400 = {&param_1400,&sp_gain_1400,&om_gain_1400};

const static t_pid_gain sp_gain_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1500 = {0.05, 0.01, 0.00};
const static t_velo_param param_1500 = {1.50f,12.0f};
const static t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};

const static t_pid_gain sp_gain_1600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1600 = {0.05, 0.01, 0.00};
const static t_velo_param param_1600 = {1.60f,12.0f};
const static t_straight_param st_param_1600 = {&param_1600,&sp_gain_1600,&om_gain_1600};

const static t_pid_gain sp_gain_1700 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1700 = {0.05, 0.01, 0.00};
const static t_velo_param param_1700 = {1.70f,12.0f};
const static t_straight_param st_param_1700 = {&param_1700,&sp_gain_1700,&om_gain_1700};

const static t_pid_gain sp_gain_1800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1800 = {0.05, 0.01, 0.00};
const static t_velo_param param_1800 = {1.80f,12.0f};
const static t_straight_param st_param_1800 = {&param_1800,&sp_gain_1800,&om_gain_1800};

const static t_pid_gain sp_gain_1900 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1900 = {0.05, 0.01, 0.00};
const static t_velo_param param_1900 = {1.90f,12.0f};
const static t_straight_param st_param_1900 = {&param_1900,&sp_gain_1900,&om_gain_1900};

const static t_pid_gain sp_gain_2000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2000 = {0.05, 0.01, 0.0};
const static t_velo_param param_2000 = {2.0f,12.0f};
const static t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};

const static t_pid_gain sp_gain_2100 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2100 = {0.05, 0.01, 0.0};
const static t_velo_param param_2100 = {2.1f,12.0f};
const static t_straight_param st_param_2100 = {&param_2100,&sp_gain_2100,&om_gain_2100};

const static t_pid_gain sp_gain_2200 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2200 = {0.05, 0.01, 0.0};
const static t_velo_param param_2200 = {2.2f,12.0f};
const static t_straight_param st_param_2200 = {&param_2200,&sp_gain_2200,&om_gain_2200};

const static t_pid_gain sp_gain_2300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2300 = {0.05, 0.01, 0.0};
const static t_velo_param param_2300 = {2.3f,12.0f};
const static t_straight_param st_param_2300 = {&param_2300,&sp_gain_2300,&om_gain_2300};

const static t_pid_gain sp_gain_2400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2400 = {0.05, 0.01, 0.0};
const static t_velo_param param_2400 = {2.4f,12.0f};
const static t_straight_param st_param_2400 = {&param_2400,&sp_gain_2400,&om_gain_2400};

const static t_pid_gain sp_gain_2500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2500 = {0.05, 0.01, 0.0};
const static t_velo_param param_2500 = {2.5f,12.0f};
const static t_straight_param st_param_2500 = {&param_2500,&sp_gain_2500,&om_gain_2500};

const static t_pid_gain sp_gain_2600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2600 = {0.05, 0.01, 0.0};
const static t_velo_param param_2600 = {2.6f,15.0f};
const static t_straight_param st_param_2600 = {&param_2600,&sp_gain_2600,&om_gain_2600};

const static t_pid_gain sp_gain_2700 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2700 = {0.05, 0.01, 0.0};
const static t_velo_param param_2700 = {2.7f,15.0f};
const static t_straight_param st_param_2700 = {&param_2700,&sp_gain_2700,&om_gain_2700};


const static t_pid_gain sp_gain_2800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2800 = {0.05, 0.01, 0.0};
const static t_velo_param param_2800 = {2.8f,15.0f};
const static t_straight_param st_param_2800 = {&param_2800,&sp_gain_2800,&om_gain_2800};


const static t_pid_gain sp_gain_2900 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2900 = {0.05, 0.01, 0.0};
const static t_velo_param param_2900 = {2.9f,15.0f};
const static t_straight_param st_param_2900 = {&param_2900,&sp_gain_2900,&om_gain_2900};

const static t_pid_gain sp_gain_3000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_3000 = {0.05, 0.01, 0.0};
const static t_velo_param param_3000 = {3.0f,15.0f};
const static t_straight_param st_param_3000 = {&param_3000,&sp_gain_3000,&om_gain_3000};



const static t_straight_param *const st_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const st_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const st_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const st_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const st_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const st_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const st_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const st_mode_1400_v0[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const st_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};

const static t_straight_param *const di_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const di_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const di_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const di_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const di_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const di_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const di_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const di_mode_1400_v0[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const di_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};

const static t_pid_gain sp_gain_dummy = {0.0f,0.0f,0.0f};
const static t_pid_gain om_gain_dummy = {0.0f, 0.0f, 0.0f};
const static t_turn_param_table slalom_dummy = {0.0f,0.0f,0.0f,0.0f,0.0f,Turn_L};
const static t_param param_dummy = {&slalom_dummy,&sp_gain_dummy,&om_gain_dummy};

const static t_pid_gain sp_gain_turn90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_300 = {0.4, 0.05, 0.0};
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
const static t_pid_gain om_gain_turnV90_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_300_table = {0.30f, 30.0f,22.78,23.50, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_300_table = {0.30f,-30.0f,22.78,23.50,-90.0f,Turn_R};
const static t_param param_LV90_300 = {&slalom_LV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};
const static t_param param_RV90_300 = {&slalom_RV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};

const static t_pid_gain sp_gain_turnIn45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_300_table = {0.30f, 30.0f,27.04,46.34, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_300_table = {0.30f,-30.0f,27.04,46.34,-45.0f,Turn_R};
const static t_param param_inL45_300 = {&slalom_inL45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};
const static t_param param_inR45_300 = {&slalom_inR45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};

const static t_pid_gain sp_gain_turnOut45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_300_table = {0.30f, 30.0f,45.68,27.70, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_300_table = {0.30f,-30.0f,45.68,27.70,-45.0f,Turn_R};
const static t_param param_outL45_300 = {&slalom_outL45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};
const static t_param param_outR45_300 = {&slalom_outR45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};

const static t_pid_gain sp_gain_turnIn135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL135_300_table = {0.30f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_300_table = {0.30f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_300 = {&slalom_inL135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};
const static t_param param_inR135_300 = {&slalom_inR135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};

const static t_pid_gain sp_gain_turnOut135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_300 = {0.4, 0.05, 0.0};
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


//-----------velo = 500 mm/s parameters
const static t_pid_gain sp_gain_turn90_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 42.5f,30.62,35.22, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_500_table = {0.50f,-42.5f,30.60,35.22,-90.0f,Turn_R};// k= 100
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,22.4,25.48, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,22.4,25.48,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 35.0f,15.23,17.27, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-35.0f,15.23,17.27,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 42.5f,18.91,39.79, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-42.5f,18.91,39.79,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.4, 0.05, 0.0};;
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 42.5f,37.59,21.13, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-42.5f,37.59,21.13,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 35.0f,29.58+4.0,24.17, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-35.0f,29.58+4.0,24.17,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 35.0f,21.87,31.90, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-35.0f,21.87,31.90,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};

const static t_param *const mode_500[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_500,		&param_L90_500,
												&param_R180_500,	&param_L180_500,
												&param_inR45_500,	&param_inL45_500,
												&param_outR45_500,	&param_outL45_500,
												&param_inR135_500,	&param_inL135_500,
												&param_outR135_500,	&param_outL135_500,
												&param_RV90_500,	&param_LV90_500
											};

//k = 200

const static t_pid_gain sp_gain_turn90_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1000_table = {1.00f, 42.5f,29.48,39.30, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1000_table = {1.00f,-42.5f,29.48,39.30,-90.0f,Turn_R};
const static t_param param_L90_1000 = {&slalom_L90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};
const static t_param param_R90_1000 = {&slalom_R90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};

const static t_pid_gain sp_gain_turn180_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1000_table = {1.00f, 45.0f,18.54,28.49, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1000_table = {1.00f,-45.0f,18.54,28.49,-180.0f,Turn_R};
const static t_param param_L180_1000 = {&slalom_L180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};
const static t_param param_R180_1000 = {&slalom_R180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};

//not adjust
const static t_pid_gain sp_gain_turnV90_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_1000_table = {1.00f, 37.5f,10.20,19.62, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1000_table = {1.00f,-37.5f,10.20,19.62,-90.0f,Turn_R};
const static t_param param_LV90_1000 = {&slalom_LV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};
const static t_param param_RV90_1000 = {&slalom_RV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};

const static t_pid_gain sp_gain_turnIn45_1000 = {6.0, 0.02, 0.0};
const static t_pid_gain om_gain_turnIn45_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_1000_table = {1.00f, 45.0f,15.52,42.41, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1000_table = {1.00f,-45.0f,15.52,42.41,-45.0f,Turn_R};
const static t_param param_inL45_1000 = {&slalom_inL45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};
const static t_param param_inR45_1000 = {&slalom_inR45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_1000_table = {1.00f, 50.0f,31.04,21.30, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1000_table = {1.00f,-50.0f,31.04,21.30,-45.0f,Turn_R};
const static t_param param_outL45_1000 = {&slalom_outL45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};
const static t_param param_outR45_1000 = {&slalom_outR45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};


const static t_pid_gain sp_gain_turnIn135_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1000 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1000_table = {1.00f, 37.5f,27.18,29.87, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1000_table = {1.00f,-37.5f,27.18,29.87,-135.0f,Turn_R};
const static t_param param_inL135_1000 = {&slalom_inL135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};
const static t_param param_inR135_1000 = {&slalom_inR135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};

//
const static t_pid_gain sp_gain_turnOut135_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1000_table = {1.00f, 38.0f,18.00,35.81, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1000_table = {1.00f,-38.0f,18.00,35.81,-135.0f,Turn_R};
const static t_param param_outL135_1000 = {&slalom_outL135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};
const static t_param param_outR135_1000 = {&slalom_outR135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};

const static t_param *const mode_1000[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1000,		&param_L90_1000,
												&param_R180_1000,	&param_L180_1000,
												&param_inR45_1000,	&param_inL45_1000,
												&param_outR45_1000,	&param_outL45_1000,
												&param_inR135_1000,	&param_inL135_1000,
												&param_outR135_1000,	&param_outL135_1000,
												&param_RV90_1000,	&param_LV90_1000
											};


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
const static t_pid_gain om_gain_turnV90_1200 = {0.2, 0.01, 0.0};
const static t_turn_param_table slalom_LV90_1200_table = {1.20f, 41.0f,5.27,17.65, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1200_table = {1.20f,-41.0f,5.27,17.65,-90.0f,Turn_R};
const static t_param param_LV90_1200 = {&slalom_LV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};
const static t_param param_RV90_1200 = {&slalom_RV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};

const static t_pid_gain sp_gain_turnIn45_1200 = {6.5, 0.02, 0.0};
const static t_pid_gain om_gain_turnIn45_1200 = {0.3, 0.01, 0.0};
const static t_turn_param_table slalom_inL45_1200_table = {1.20f, 48.0f,12.20,42.94, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1200_table = {1.20f,-48.0f,12.20,42.94,-45.0f,Turn_R};
const static t_param param_inL45_1200 = {&slalom_inL45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};
const static t_param param_inR45_1200 = {&slalom_inR45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1200 = {6.5, 0.02, 0.0};
const static t_pid_gain om_gain_turnOut45_1200 = {0.3, 0.01, 0.0};
const static t_turn_param_table slalom_outL45_1200_table = {1.20f, 45.0f,32.75,26.20, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1200_table = {1.20f,-45.0f,32.75,26.20,-45.0f,Turn_R};
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

const static t_param *const mode_1200[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1200,		&param_L90_1200,
												&param_R180_1200,	&param_L180_1200,
												&param_inR45_1200,	&param_inL45_1200,
												&param_outR45_1200,	&param_outL45_1200,
												&param_inR135_1200,	&param_inL135_1200,
												&param_outR135_1200,	&param_outL135_1200,
												&param_RV90_1200,	&param_LV90_1200
											};

const static t_pid_gain sp_gain_turn90_1400 = {7.50, 0.01, 0.0};
const static t_pid_gain om_gain_turn90_1400 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,18.83,36.69, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,18.83,36.69,-90.0f,Turn_R};
const static t_param param_L90_1400 = {&slalom_L90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};
const static t_param param_R90_1400 = {&slalom_R90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};

const static t_pid_gain sp_gain_turn180_1400 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_turn180_1400 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.50f,11.70,28.82, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.50f,11.70,28.82,-180.0f,Turn_R};
const static t_param param_L180_1400 = {&slalom_L180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};
const static t_param param_R180_1400 = {&slalom_R180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};

//not adjust
const static t_pid_gain sp_gain_turnV90_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnV90_1400 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,9.57,26.82, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,9.57,26.82,-90.0f,Turn_R};
const static t_param param_LV90_1400 = {&slalom_LV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};
const static t_param param_RV90_1400 = {&slalom_RV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};

const static t_pid_gain sp_gain_turnIn45_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnIn45_1400 = {0.3, 0.02, 0.0};
const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 52.0f,8.258,42.69, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-52.0f,8.258,42.69,-45.0f,Turn_R};
const static t_param param_inL45_1400 = {&slalom_inL45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};
const static t_param param_inR45_1400 = {&slalom_inR45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnOut45_1400 = {0.3, 0.02, 0.0};
const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 48.0f,29.63,26.45, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-48.0f,29.63,26.45,-45.0f,Turn_R};
const static t_param param_outL45_1400 = {&slalom_outL45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};
const static t_param param_outR45_1400 = {&slalom_outR45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};


const static t_pid_gain sp_gain_turnIn135_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnIn135_1400 = {0.4, 0.02, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 41.0f,17.38,28.48, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-41.0f,17.38,28.48,-135.0f,Turn_R};
const static t_param param_inL135_1400 = {&slalom_inL135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};
const static t_param param_inR135_1400 = {&slalom_inR135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};

//
const static t_pid_gain sp_gain_turnOut135_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnOut135_1400 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 41.0f,12.78,40.26, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-41.0f,12.78,40.26,-135.0f,Turn_R};
const static t_param param_outL135_1400 = {&slalom_outL135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};
const static t_param param_outR135_1400 = {&slalom_outR135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};

const static t_param *const mode_1400[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400
											};
/*
const static t_pid_gain sp_gain_turn90_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1500_table = {1.50f, 55.5f,11.58,32.07, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1500_table = {1.50f,-55.5f,11.58,32.07,-90.0f,Turn_R};
const static t_param param_L90_1500 = {&slalom_L90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};
const static t_param param_R90_1500 = {&slalom_R90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};

const static t_pid_gain sp_gain_turn180_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1500_table = {1.50f, 49.50f,9.64,28.80, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1500_table = {1.50f,-49.50f,9.64,28.80,-180.0f,Turn_R};
const static t_param param_L180_1500 = {&slalom_L180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};
const static t_param param_R180_1500 = {&slalom_R180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};

//not adjust
const static t_pid_gain sp_gain_turnV90_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_1500_table = {1.50f, 42.50f,3.64,23.15, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1500_table = {1.50f,-42.50f,3.64,23.15,-90.0f,Turn_R};
const static t_param param_LV90_1500 = {&slalom_LV90_1500_table,&sp_gain_turnV90_1500,&om_gain_turnV90_1500};
const static t_param param_RV90_1500 = {&slalom_RV90_1500_table,&sp_gain_turnV90_1500,&om_gain_turnV90_1500};

const static t_pid_gain sp_gain_turnIn45_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_1500_table = {1.50f, 50.0f,8.93,45.03, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1500_table = {1.50f,-50.0f,8.93,45.03,-45.0f,Turn_R};
const static t_param param_inL45_1500 = {&slalom_inL45_1500_table,&sp_gain_turnIn45_1500,&om_gain_turnIn45_1500};
const static t_param param_inR45_1500 = {&slalom_inR45_1500_table,&sp_gain_turnIn45_1500,&om_gain_turnIn45_1500};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_1500_table = {1.50f, 50.0f,27.61,26.35, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1500_table = {1.50f,-50.0f,27.61,26.35,-45.0f,Turn_R};
const static t_param param_outL45_1500 = {&slalom_outL45_1500_table,&sp_gain_turnOut45_1500,&om_gain_turnOut45_1500};
const static t_param param_outR45_1500 = {&slalom_outR45_1500_table,&sp_gain_turnOut45_1500,&om_gain_turnOut45_1500};


const static t_pid_gain sp_gain_turnIn135_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1500 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1500_table = {1.50f, 42.0f,18.07,31.20, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1500_table = {1.50f,-42.0f,18.07,31.20,-135.0f,Turn_R};
const static t_param param_inL135_1500 = {&slalom_inL135_1500_table,&sp_gain_turnIn135_1500,&om_gain_turnIn135_1500};
const static t_param param_inR135_1500 = {&slalom_inR135_1500_table,&sp_gain_turnIn135_1500,&om_gain_turnIn135_1500};

//
const static t_pid_gain sp_gain_turnOut135_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_1500 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1500_table = {1.50f, 42.0f,10.45,39.01, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1500_table = {1.50f,-42.0f,10.45,39.01,-135.0f,Turn_R};
const static t_param param_outL135_1500 = {&slalom_outL135_1500_table,&sp_gain_turnOut135_1500,&om_gain_turnOut135_1500};
const static t_param param_outR135_1500 = {&slalom_outR135_1500_table,&sp_gain_turnOut135_1500,&om_gain_turnOut135_1500};

const static t_param *const mode_1500[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1500,		&param_L90_1500,
												&param_R180_1500,	&param_L180_1500,
												&param_inR45_1500,	&param_inL45_1500,
												&param_outR45_1500,	&param_outL45_1500,
												&param_inR135_1500,	&param_inL135_1500,
												&param_outR135_1500,	&param_outL135_1500,
												&param_RV90_1500,	&param_LV90_1500
											};
*/
#endif /* CPP_INC_RUN_PARAM_H_ */
