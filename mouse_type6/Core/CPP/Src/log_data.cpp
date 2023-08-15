/*
 * log_data.cpp
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */



#include "../../Module/Include/index.h"
#include "log_data.h"
#include "stdio.h"

void LogData::indicate_data()
{
	for(int i = 0; i< 1000;i++)
	{
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",data[0][i],data[1][i],data[2][i],data[3][i]);
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",data[4][i],data[5][i],data[6][i],data[7][i]);
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf\n",data[8][i],data[9][i],data[10][i],data[11][i]);
		HAL_Delay(2);
	}
}

