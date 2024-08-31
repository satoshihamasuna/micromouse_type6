/*
 * run_pram.h
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_PARAM_H_
#define CPP_INC_RUN_PARAM_H_


#include "../Component/Inc/controller.h"

#include "../Module/Inc/vehicle.h"

#define TURN_MODES (19)

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
}t_turn_param_table;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
}t_velo_param;

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_param;


typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_straight_param;

const static t_pid_gain basic_sp_gain = {6.0, 0.05, 0.0};
const static t_pid_gain basic_om_gain = {0.35  , 0.01, 0.0};

const static t_pid_gain search_sp_gain = {6.0, 0.05, 0.0};
const static t_pid_gain search_om_gain = {0.35, 0.01, 0.0};

const static t_pid_gain sp_gain_search_turn = {6.0, 0.05, 0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.4, 0.01, 0.0};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_table = {0.30f, 25.0f,10.45,12.95, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table = {0.30f,-25.0f,10.45,12.95,-90.0f,Turn_R};
const static t_param param_L90_search = {&slalom_L90_table ,&sp_gain_search_turn,&om_gain_search_turn};
const static t_param param_R90_search = {&slalom_R90_table, &sp_gain_search_turn,&om_gain_search_turn};

const static t_pid_gain sp_gain_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_300 = {0.3, 0.01, 0.00};
const static t_velo_param param_300 = {0.30f,4.0f};
const static t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300};

const static t_pid_gain sp_gain_450 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_450 = {0.4, 0.01, 0.00};
const static t_velo_param param_450 = {0.45f,6.0f};
const static t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450};

const static t_pid_gain sp_gain_500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_500 = {0.4, 0.01, 0.00};
const static t_velo_param param_500 = {0.50f,6.0f};
const static t_straight_param st_param_500 = {&param_500,&sp_gain_500,&om_gain_500};

const static t_pid_gain sp_gain_600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_600 = {0.4, 0.01, 0.00};
const static t_velo_param param_600 = {0.60f,6.0f};
const static t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600};

const static t_velo_param param_600_acc7 = {0.60f,7.0f};
const static t_straight_param st_param_600_acc7 = {&param_600_acc7,&sp_gain_600,&om_gain_600};

const static t_pid_gain sp_gain_650 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_650 = {0.2, 0.001, 0.00};
const static t_velo_param param_650 = {0.65f,6.0f};
const static t_straight_param st_param_650 = {&param_650,&sp_gain_650,&om_gain_650};

const static t_velo_param param_650_acc7 = {0.65f,7.0f};
const static t_straight_param st_param_650_acc7 = {&param_650_acc7,&sp_gain_650,&om_gain_650};

const static t_pid_gain sp_gain_700 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_700 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_700_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_700_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_700 = {0.70f,6.0f};
const static t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700};

const static t_velo_param param_700_acc7 = {0.70f,7.0f};
const static t_straight_param st_param_700_acc7 = {&param_700_acc7,&sp_gain_700_acc7,&om_gain_700_acc7};

const static t_pid_gain sp_gain_800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_800 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_800_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_800_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_800 = {0.80f,7.0f};
const static t_straight_param st_param_800 = {&param_800,&sp_gain_800,&om_gain_800};
const static t_straight_param st_param_800_acc7 = {&param_800,&sp_gain_800_acc7,&om_gain_800_acc7};

const static t_pid_gain sp_gain_900 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_900 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_900_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_900_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_900 = {0.90f,7.0f};
const static t_straight_param st_param_900 = {&param_900,&sp_gain_900,&om_gain_900};
const static t_straight_param st_param_900_acc7 = {&param_900,&sp_gain_900_acc7,&om_gain_900_acc7};

const static t_pid_gain sp_gain_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1000 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_1000_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1000_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_1000 = {1.0f,9.0f};
const static t_velo_param param_1000_acc7 = {1.0f,7.0f};
const static t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};
const static t_straight_param st_param_1000_acc7 = {&param_1000_acc7,&sp_gain_1000_acc7,&om_gain_1000_acc7};

const static t_pid_gain sp_gain_1050 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1050 = {0.4, 0.01, 0.00};
const static t_velo_param param_1050 = {1.05f,9.0f};
const static t_straight_param st_param_1050 = {&param_1050,&sp_gain_1050,&om_gain_1050};

const static t_pid_gain sp_gain_1100 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1100 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_1100_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1100_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_1100 = {1.10f,9.0f};
const static t_velo_param param_1100_acc7 = {1.1f,7.0f};
const static t_straight_param st_param_1100 = {&param_1100,&sp_gain_1100,&om_gain_1100};
const static t_straight_param st_param_1100_acc7 = {&param_1100_acc7,&sp_gain_1100_acc7,&om_gain_1100_acc7};

const static t_pid_gain sp_gain_1200 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1200 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_1200_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1200_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_1200 = {1.20f,10.0f};
const static t_velo_param param_1200_acc7 = {1.2f,7.0f};
const static t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};
const static t_straight_param st_param_1200_acc7 = {&param_1200_acc7,&sp_gain_1200_acc7,&om_gain_1200_acc7};

const static t_pid_gain sp_gain_1300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1300 = {0.4, 0.01, 0.00};
const static t_pid_gain sp_gain_1300_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1300_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_1300 = {1.30f,10.0f};
const static t_velo_param param_1300_acc7 = {1.3f,7.0f};
const static t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};
const static t_straight_param st_param_1300_acc7 = {&param_1300_acc7,&sp_gain_1300_acc7,&om_gain_1300_acc7};

const static t_pid_gain sp_gain_1400 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_1400 = {0.2, 0.01, 0.00};
const static t_pid_gain sp_gain_1400_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1400_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_1400 = {1.40f,12.0f};
const static t_velo_param param_1400_acc7 = {1.4f,7.0f};
const static t_velo_param param_1400_acc16 = {1.40f,16.0f};
const static t_velo_param param_1400_acc20 = {1.40f,20.0f};
const static t_straight_param st_param_1400 = {&param_1400,&sp_gain_1400,&om_gain_1400};
const static t_straight_param st_param_1400_acc7 = {&param_1400_acc7,&sp_gain_1400_acc7,&om_gain_1400_acc7};
const static t_straight_param st_param_1400_acc16 = {&param_1400_acc16,&sp_gain_1400,&om_gain_1400};
const static t_straight_param st_param_1400_acc20 = {&param_1400_acc20,&sp_gain_1400,&om_gain_1400};

const static t_pid_gain sp_gain_1500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1500 = {0.2, 0.01, 0.00};
const static t_pid_gain sp_gain_1500_acc7 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1500_acc7 = {0.2, 0.001, 0.00};
const static t_velo_param param_1500 = {1.50f,12.0f};
const static t_velo_param param_1500_acc7 = {1.5f,7.0f};
const static t_velo_param param_1500_acc16 = {1.50f,16.0f};
const static t_velo_param param_1500_acc20 = {1.50f,20.0f};
const static t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};
const static t_straight_param st_param_1500_acc7 = {&param_1500_acc7,&sp_gain_1500_acc7,&om_gain_1500_acc7};
const static t_straight_param st_param_1500_acc16 = {&param_1500_acc16,&sp_gain_1500,&om_gain_1500};
const static t_straight_param st_param_1500_acc20 = {&param_1500_acc20,&sp_gain_1500,&om_gain_1500};

const static t_pid_gain sp_gain_1600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1600 = {0.2, 0.01, 0.00};
const static t_velo_param param_1600 = {1.60f,12.0f};
const static t_velo_param param_1600_acc7 = {1.6f,7.0f};
const static t_velo_param param_1600_acc16 = {1.60f,16.0f};
const static t_velo_param param_1600_acc20 = {1.60f,20.0f};
const static t_straight_param st_param_1600 = {&param_1600,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc7 = {&param_1600_acc7,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc16 = {&param_1600_acc16,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc20 = {&param_1600_acc20,&sp_gain_1600,&om_gain_1600};

const static t_pid_gain sp_gain_1700 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1700 = {0.2, 0.01, 0.00};
const static t_velo_param param_1700 = {1.70f,12.0f};
const static t_velo_param param_1700_acc7 = {1.7f,7.0f};
const static t_velo_param param_1700_acc16 = {1.60f,16.0f};
const static t_velo_param param_1700_acc20 = {1.60f,20.0f};
const static t_straight_param st_param_1700 = {&param_1700,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc7 = {&param_1700_acc7,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc16 = {&param_1700_acc16,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc20 = {&param_1700_acc20,&sp_gain_1700,&om_gain_1700};

const static t_pid_gain sp_gain_1800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1800 = {0.4, 0.01, 0.00};
const static t_velo_param param_1800 = {1.80f,12.0f};
const static t_velo_param param_1800_acc7 = {1.8f,7.0f};
const static t_velo_param param_1800_acc16 = {1.80f,16.0f};
const static t_velo_param param_1800_acc20 = {1.80f,20.0f};
const static t_straight_param st_param_1800 = {&param_1800,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc7 = {&param_1800_acc7,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc16 = {&param_1800_acc16,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc20 = {&param_1800_acc20,&sp_gain_1800,&om_gain_1800};

const static t_pid_gain sp_gain_1900 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_1900 = {0.4, 0.01, 0.00};
const static t_velo_param param_1900 = {1.90f,12.0f};
const static t_velo_param param_1900_acc7 = {1.9f,7.0f};
const static t_velo_param param_1900_acc16 = {1.90f,16.0f};
const static t_straight_param st_param_1900 = {&param_1900,&sp_gain_1900,&om_gain_1900};
const static t_straight_param st_param_1900_acc7 = {&param_1900_acc7,&sp_gain_1900,&om_gain_1900};
const static t_straight_param st_param_1900_acc16 = {&param_1900_acc16,&sp_gain_1900,&om_gain_1900};


const static t_pid_gain sp_gain_2000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2000 = {0.2, 0.01, 0.00};
const static t_velo_param param_2000 = {2.0f,12.0f};
const static t_velo_param param_2000_acc7 = {2.0f,7.0f};
const static t_velo_param param_2000_acc16 = {2.00f,16.0f};
const static t_velo_param param_2000_acc20 = {2.00f,20.0f};
const static t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc7 = {&param_2000_acc7,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc16 = {&param_2000_acc16,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc20 = {&param_2000_acc20,&sp_gain_2000,&om_gain_2000};

const static t_pid_gain sp_gain_2100 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2100 = {0.2, 0.01, 0.00};
const static t_velo_param param_2100 = {2.1f,12.0f};
const static t_velo_param param_2100_acc16 = {2.10f,16.0f};
const static t_velo_param param_2100_acc20 = {2.10f,20.0f};
const static t_straight_param st_param_2100 = {&param_2100,&sp_gain_2100,&om_gain_2100};
const static t_straight_param st_param_2100_acc16 = {&param_2100_acc16,&sp_gain_2100,&om_gain_2100};
const static t_straight_param st_param_2100_acc20 = {&param_2100_acc20,&sp_gain_2100,&om_gain_2100};

const static t_pid_gain sp_gain_2200 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2200 = {0.2, 0.01, 0.00};
const static t_velo_param param_2200 = {2.2f,12.0f};
const static t_velo_param param_2200_acc16 = {2.20f,16.0f};
const static t_velo_param param_2200_acc20 = {2.20f,20.0f};
const static t_straight_param st_param_2200 = {&param_2200,&sp_gain_2200,&om_gain_2200};
const static t_straight_param st_param_2200_acc16 = {&param_2200_acc16,&sp_gain_2200,&om_gain_2200};
const static t_straight_param st_param_2200_acc20 = {&param_2200_acc20,&sp_gain_2200,&om_gain_2200};

const static t_pid_gain sp_gain_2300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2300 = {0.2, 0.01, 0.00};
const static t_velo_param param_2300 = {2.3f,12.0f};
const static t_velo_param param_2300_acc16 = {2.30f,16.0f};
const static t_velo_param param_2300_acc20 = {2.30f,20.0f};
const static t_straight_param st_param_2300 = {&param_2300,&sp_gain_2300,&om_gain_2300};
const static t_straight_param st_param_2300_acc16 = {&param_2300_acc16,&sp_gain_2300,&om_gain_2300};
const static t_straight_param st_param_2300_acc20 = {&param_2300_acc20,&sp_gain_2300,&om_gain_2300};

const static t_pid_gain sp_gain_2400 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2400 = {0.2, 0.01, 0.00};
const static t_velo_param param_2400 = {2.4f,12.0f};
const static t_velo_param param_2400_acc16 = {2.40f,16.0f};
const static t_velo_param param_2400_acc20 = {2.40f,20.0f};
const static t_straight_param st_param_2400 = {&param_2400,&sp_gain_2400,&om_gain_2400};
const static t_straight_param st_param_2400_acc16 = {&param_2400_acc16,&sp_gain_2400,&om_gain_2400};
const static t_straight_param st_param_2400_acc20 = {&param_2400_acc20,&sp_gain_2400,&om_gain_2400};

const static t_pid_gain sp_gain_2500 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2500 = {0.2, 0.01, 0.00};
const static t_velo_param param_2500 = {2.5f,12.0f};
const static t_velo_param param_2500_acc16 = {2.50f,16.0f};
const static t_velo_param param_2500_acc20 = {2.50f,20.0f};
const static t_straight_param st_param_2500 = {&param_2500,&sp_gain_2500,&om_gain_2500};
const static t_straight_param st_param_2500_acc16 = {&param_2500_acc16,&sp_gain_2500,&om_gain_2500};
const static t_straight_param st_param_2500_acc20 = {&param_2500_acc20,&sp_gain_2500,&om_gain_2500};

const static t_pid_gain sp_gain_2600 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2600 = {0.2, 0.01, 0.00};
const static t_velo_param param_2600 = {2.6f,15.0f};
const static t_velo_param param_2600_acc16 = {2.60f,16.0f};
const static t_velo_param param_2600_acc20 = {2.60f,20.0f};
const static t_straight_param st_param_2600 = {&param_2600,&sp_gain_2600,&om_gain_2600};
const static t_straight_param st_param_2600_acc16 = {&param_2600_acc16,&sp_gain_2600,&om_gain_2600};
const static t_straight_param st_param_2600_acc20 = {&param_2600_acc20,&sp_gain_2600,&om_gain_2600};

const static t_pid_gain sp_gain_2700 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2700 = {0.2, 0.01, 0.00};
const static t_velo_param param_2700 = {2.7f,15.0f};
const static t_straight_param st_param_2700 = {&param_2700,&sp_gain_2700,&om_gain_2700};
const static t_velo_param param_2700_acc16 = {2.70f,16.0f};
const static t_straight_param st_param_2700_acc16 = {&param_2700_acc16,&sp_gain_2700,&om_gain_2700};

const static t_pid_gain sp_gain_2800 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2800 = {0.2, 0.01, 0.00};
const static t_velo_param param_2800 = {2.8f,15.0f};
const static t_velo_param param_2800_acc16 = {2.80f,16.0f};
const static t_velo_param param_2800_acc20 = {2.80f,20.0f};
const static t_straight_param st_param_2800 = {&param_2800,&sp_gain_2800,&om_gain_2800};
const static t_straight_param st_param_2800_acc16 = {&param_2800_acc16,&sp_gain_2800,&om_gain_2800};
const static t_straight_param st_param_2800_acc20 = {&param_2800_acc20,&sp_gain_2800,&om_gain_2800};

const static t_pid_gain sp_gain_2900 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_2900 = {0.2, 0.01, 0.00};
const static t_velo_param param_2900 = {2.9f,15.0f};
const static t_velo_param param_2900_acc16 = {2.90f,16.0f};
const static t_velo_param param_2900_acc20 = {2.90f,20.0f};
const static t_straight_param st_param_2900 = {&param_2900,&sp_gain_2900,&om_gain_2900};
const static t_straight_param st_param_2900_acc16 = {&param_2900_acc16,&sp_gain_2900,&om_gain_2900};
const static t_straight_param st_param_2900_acc20 = {&param_2900_acc20,&sp_gain_2900,&om_gain_2900};


const static t_pid_gain sp_gain_3000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_3000 = {0.2, 0.01, 0.00};
const static t_velo_param param_3000 = {3.0f,15.0f};
const static t_velo_param param_3000_acc16 = {3.00f,16.0f};
const static t_velo_param param_3000_acc20 = {3.00f,20.0f};
const static t_straight_param st_param_3000 = {&param_3000,&sp_gain_3000,&om_gain_3000};
const static t_straight_param st_param_3000_acc16 = {&param_3000_acc16,&sp_gain_3000,&om_gain_3000};
const static t_straight_param st_param_3000_acc20 = {&param_3000_acc20,&sp_gain_3000,&om_gain_3000};

const static t_pid_gain sp_gain_3200 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_3200 = {0.2, 0.01, 0.00};
const static t_velo_param param_3200 = {3.2f,15.0f};
const static t_velo_param param_3200_acc16 = {3.20f,16.0f};
const static t_velo_param param_3200_acc20 = {3.20f,20.0f};
const static t_straight_param st_param_3200 = {&param_3200,&sp_gain_3200,&om_gain_3200};
const static t_straight_param st_param_3200_acc16 = {&param_3200_acc16,&sp_gain_3200,&om_gain_3200};
const static t_straight_param st_param_3200_acc20 = {&param_3200_acc20,&sp_gain_3200,&om_gain_3200};

const static t_straight_param *const st_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const st_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const st_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};

const static t_straight_param *const st_mode_500_v1[] = {	&st_param_500,			&st_param_600_acc7, 	&st_param_700_acc7,		&st_param_800_acc7,		&st_param_900_acc7,		&st_param_1000_acc7,
															&st_param_1100_acc7,	&st_param_1200_acc7,	&st_param_1300_acc7,	&st_param_1400_acc7,	&st_param_1500_acc7};
const static t_straight_param *const st_mode_650_v0[] = {&st_param_650_acc7, 	&st_param_700_acc7,		&st_param_800_acc7,		&st_param_900_acc7,		&st_param_1000_acc7,
														 &st_param_1100_acc7,	&st_param_1200_acc7,	&st_param_1300_acc7,	&st_param_1400_acc7,	&st_param_1500_acc7};
const static t_straight_param *const st_mode_800_v0[] = {&st_param_800_acc7,	&st_param_900_acc7,		&st_param_1000_acc7,	&st_param_1100_acc7,	&st_param_1200_acc7,
														 &st_param_1300_acc7,	&st_param_1400_acc7,	&st_param_1500_acc7,	&st_param_1600_acc7,	&st_param_1700_acc7,
														 &st_param_1800_acc7,	&st_param_1900_acc7,	&st_param_2000_acc7};

const static t_straight_param *const st_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const st_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const st_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const st_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const st_mode_1400_v0[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const st_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const st_mode_1400_v2[] = {	&st_param_1400_acc16,&st_param_1600_acc16,&st_param_1800_acc16,&st_param_2000_acc16,
															&st_param_2200_acc16,&st_param_2400_acc16,&st_param_2600_acc16,&st_param_2800_acc16,
															&st_param_3000_acc16	};
const static t_straight_param *const st_mode_1400_v3[] = {	&st_param_1400_acc20,&st_param_1600_acc20,&st_param_1800_acc20,&st_param_2000_acc20,
															&st_param_2200_acc20,&st_param_2400_acc20,&st_param_2600_acc20,&st_param_2800_acc20,
															&st_param_3000_acc20	};


const static t_straight_param *const di_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const di_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const di_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const di_mode_500_v1[] = {	&st_param_500,			&st_param_600_acc7, 	&st_param_700_acc7,		&st_param_800_acc7,		&st_param_900_acc7,		&st_param_1000_acc7,
															&st_param_1100_acc7,	&st_param_1200_acc7,	&st_param_1300_acc7,	&st_param_1400_acc7,	&st_param_1500_acc7};

const static t_straight_param *const di_mode_650_v0[] = {	&st_param_650_acc7, 	&st_param_700_acc7,		&st_param_800_acc7,		&st_param_900_acc7,		&st_param_1000_acc7,
															&st_param_1100_acc7,	&st_param_1200_acc7,	&st_param_1300_acc7,	&st_param_1400_acc7,	&st_param_1500_acc7};
const static t_straight_param *const di_mode_800_v0[] = {&st_param_800_acc7,	&st_param_900_acc7,		&st_param_1000_acc7,	&st_param_1100_acc7,	&st_param_1200_acc7,
														 &st_param_1300_acc7,	&st_param_1400_acc7,	&st_param_1500_acc7,	&st_param_1600_acc7,	&st_param_1700_acc7,
														 &st_param_1800_acc7,	&st_param_1900_acc7,	&st_param_2000_acc7};

const static t_straight_param *const di_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const di_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const di_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const di_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const di_mode_1400_v0[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const di_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1400_v2[] = {	&st_param_1400_acc16,&st_param_1600_acc16,&st_param_1800_acc16,&st_param_2000_acc16,
															&st_param_2200_acc16,&st_param_2400_acc16,&st_param_2600_acc16,&st_param_2800_acc16,
															&st_param_3000_acc16	};

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

const static t_param *const mode_300[TURN_MODES] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
														&param_R90_300,		&param_L90_300,
														&param_R180_300,	&param_L180_300,
														&param_inR45_300,	&param_inL45_300,
														&param_outR45_300,	&param_outL45_300,
														&param_inR135_300,	&param_inL135_300,
														&param_outR135_300,	&param_outL135_300,
														&param_RV90_300,	&param_LV90_300
													};



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

//-----------velo = 650 mm/s parameters
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

const static t_pid_gain sp_gain_long_turnV90_1000 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_long_turnV90_1000 = {0.4, 0.01, 0.0};
const static t_turn_param_table slalom_LongLV90_1000_table = {1.00f, 70.0f,27.82,46.56, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_1000_table = {1.00f,-70.0f,27.82,46.56,-90.0f,Turn_R};
const static t_param param_LongLV90_1000 = {&slalom_LongLV90_1000_table,&sp_gain_long_turnV90_1000,&om_gain_long_turnV90_1000};
const static t_param param_LongRV90_1000 = {&slalom_LongRV90_1000_table,&sp_gain_long_turnV90_1000,&om_gain_long_turnV90_1000};

const static t_param *const mode_1000[TURN_MODES] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
															&param_R90_1000,		&param_L90_1000,
															&param_R180_1000,		&param_L180_1000,
															&param_inR45_1000,		&param_inL45_1000,
															&param_outR45_1000,		&param_outL45_1000,
															&param_inR135_1000,		&param_inL135_1000,
															&param_outR135_1000,	&param_outL135_1000,
															&param_RV90_1000,		&param_LV90_1000,
															&param_LongRV90_1000,	&param_LongLV90_1000
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
															&param_LongRV90_1200,	&param_LongLV90_1200
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
const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 39.0f,4.98,30.64, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-39.0f,4.98,30.64,-90.0f,Turn_R};
const static t_param param_LV90_1400 = {&slalom_LV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};
const static t_param param_RV90_1400 = {&slalom_RV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};

const static t_pid_gain sp_gain_turnIn45_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnIn45_1400 = {0.3, 0.01, 0.0};
const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 55.0f,7.18,42.23, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,7.18,42.23,-45.0f,Turn_R};
const static t_param param_inL45_1400 = {&slalom_inL45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};
const static t_param param_inR45_1400 = {&slalom_inR45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1400 = {7.5, 0.01, 0.0};
const static t_pid_gain om_gain_turnOut45_1400 = {0.3, 0.02, 0.0};
const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,19.32,24.06, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,19.32,24.06,-45.0f,Turn_R};
const static t_param param_outL45_1400 = {&slalom_outL45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};
const static t_param param_outR45_1400 = {&slalom_outR45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};


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


const static t_param *const mode_1400[TURN_MODES] = 	{	&param_dummy,			&param_dummy,		&param_dummy,
															&param_R90_1400,		&param_L90_1400,
															&param_R180_1400,		&param_L180_1400,
															&param_inR45_1400,		&param_inL45_1400,
															&param_outR45_1400,		&param_outL45_1400,
															&param_inR135_1400,		&param_inL135_1400,
															&param_outR135_1400,	&param_outL135_1400,
															&param_RV90_1400,		&param_LV90_1400,
															&param_LongRV90_1400,	&param_LongLV90_1400
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


const static t_pid_gain sp_gain_turn90_1600 = {7.50, 0.01, 0.0};
const static t_pid_gain om_gain_turn90_1600 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,13.46,32.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,13.46,32.80,-90.0f,Turn_R};
const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};

const static t_pid_gain sp_gain_turn180_1600 = {6.0, 0.01, 0.0};
const static t_pid_gain om_gain_turn180_1600 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 46.0f,12.93,35.11, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-46.0f,12.93,35.11,-180.0f,Turn_R};
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
const static t_turn_param_table slalom_LongLV90_1600_table = {1.60f, 76.0f,17.88,36.96, 90.0f,Turn_L};
const static t_turn_param_table slalom_LongRV90_1600_table = {1.60f,-76.0f,17.88,36.96,-90.0f,Turn_R};
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
															&param_LongRV90_1600,	&param_LongLV90_1600
														};




const static t_param *const *const acc_mode_500[] = {mode_500_v2,mode_650_v2};
const static t_param *const *const acc_mode_1200[] = {mode_1200_v2,mode_1400_v2};
const static t_param *const *const acc_mode_1400[] = {mode_1400_v3,mode_1600};


#endif /* CPP_INC_RUN_PARAM_H_ */
