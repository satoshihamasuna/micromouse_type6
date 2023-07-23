/*
 * half_float.h
 *
 *  Created on: 2023/07/23
 *      Author: sato1
 */

#ifndef CPP_INC_HALF_FLOAT_H_
#define CPP_INC_HALF_FLOAT_H_

#include <stdint.h>

// 16-bit half-precision floating-point representation
typedef union {
    uint16_t u;
    struct {
        uint16_t mantissa : 10;
        uint16_t exponent : 5;
        uint16_t sign : 1;
    } parts;
} half_float;

// Function to convert a 32-bit float to a 16-bit half float
half_float float_to_half(float f) {
    half_float hf;
    uint32_t bits = *((uint32_t *)&f);
    uint16_t sign = (bits >> 31) & 0x1;
    uint16_t exponent = ((bits >> 23) & 0xFF) - 127;
    uint32_t mantissa = bits & 0x7FFFFF;

    // Handle special cases: zero, infinity, NaN
    if (exponent == -127) {
        hf.parts.exponent = 0;
        hf.parts.mantissa = 0;
    } else if (exponent == 128) {
        hf.parts.exponent = 31;
        hf.parts.mantissa = 0;
    } else {
        hf.parts.exponent = exponent + 15;
        hf.parts.mantissa = mantissa >> 13;
    }

    hf.parts.sign = sign;
    return hf;
}

// Function to convert a 16-bit half float to a 32-bit float
float half_to_float(half_float hf) {
    uint16_t sign = hf.parts.sign;
    int16_t exponent = hf.parts.exponent - 15;
    uint32_t mantissa = hf.parts.mantissa << 13;
    uint32_t bits = (sign << 31) | ((exponent + 127) << 23) | mantissa;
    return *((float *)&bits);
}



#endif /* CPP_INC_HALF_FLOAT_H_ */
