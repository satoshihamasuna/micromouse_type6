/*
 * log_data.cpp
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */



#include "index.h"
#include "log_data.h"
#include "stdio.h"

void LogData::indicate_data()
{
	for(int i = 0; i< 1000;i++)
	{
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf\n",data[0][i],data[1][i],data[2][i],data[3][i]);
		HAL_Delay(2);
	}
}

