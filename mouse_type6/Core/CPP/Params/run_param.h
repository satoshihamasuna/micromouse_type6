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
#include "typedef_run_param.h"
#include "turn_300.h"
#include "turn_500.h"
#include "turn_650.h"
#include "turn_800.h"
#include "turn_1000.h"
#include "turn_1200.h"
#include "turn_1400.h"
#include "turn_1500.h"
#include "turn_1600.h"


const static t_pid_gain basic_sp_gain = {6.0, 0.05, 0.0};
const static t_pid_gain basic_om_gain = {0.35  , 0.01, 0.0};

const static t_pid_gain search_sp_gain = {6.0, 0.05, 0.0};
const static t_pid_gain search_om_gain = {0.35, 0.01, 0.0};

const static t_pid_gain sp_gain_search_turn = {6.0, 0.05, 0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.45, 0.01, 0.0};//{0.50f, 0.0005f, 0.001f};
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


const static t_param *const *const acc_mode_500[] = {mode_500_v2,mode_650_v2};
const static t_param *const *const acc_mode_1200[] = {mode_1200_v2,mode_1400_v2};
const static t_param *const *const acc_mode_1400[] = {mode_1400_v3,mode_1600};


#endif /* CPP_INC_RUN_PARAM_H_ */
