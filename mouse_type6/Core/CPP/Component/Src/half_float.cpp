/*
 * half_float.cpp
 *
 *  Created on: 2024/03/06
 *      Author: sato1
 */
#include "../Inc/half_float.h"
//#include "../codegen/rtwhalf.h"
#include <stdint.h>
#include <math.h>


float half_to_float(half_float hf)
{
	return halfToFloat(hf);
}

half_float float_to_half(float f)
{
	return floatToHalf(f);
}

