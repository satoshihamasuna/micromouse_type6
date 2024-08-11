/*
 * half_float.h
 *
 *  Created on: 2023/07/23
 *      Author: sato1
 */

#ifndef CPP_INC_HALF_FLOAT_H_
#define CPP_INC_HALF_FLOAT_H_

#include <stdint.h>
#include "../codegen/rtwhalf.h"

half_float float_to_half(float f) ;
float half_to_float(half_float hf);

#endif /* CPP_INC_HALF_FLOAT_H_ */
