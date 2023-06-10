/*
 * macro.h
 *
 *  Created on: Jun 6, 2023
 *      Author: sato1
 */

#ifndef MODULE_INC_MACRO_H_
#define MODULE_INC_MACRO_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_MODE	0
#define ENABLE_MODE1   0x10
#define ENABLE_MODE2   0x20
#define ENABLE_MODE3   0x30
#define DISENABLE_MODE 0x00

#define ENC_RESOLUTION	(1024)

#define MOUSE_ENABLE	(0x10)

#define STRAIGHT_MODE	(1)
#define TURN_MODE		(2)
#define TURN_MODE_TABLE (4)
#define DIAG_MODE		(3)
#define NON_CON_MODE	(0)


#define G					(9.80665f)					// 重量加速度[m/s^2]
#define PI					(3.1415926f)				// 円周率
#define SQRT2				(1.41421356237f)			// ルート2
#define SQRT3				(1.73205080757f)			// ルート3
#define SQRT5				(2.2360679775f)				// ルート5
#define SQRT7				(2.64575131106f)			// ルート7

#define DEG2RAD(x)			(((x)/180.0f)*PI)			// 度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI))			// ラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))
#define ABS(x) 				((x) < 0 ? -(x) : (x))		// 絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			// 符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))

//machine parameter
#define SECTION				(90.0)
#define HALF_SECTION		(SECTION/2.0)
#define DIAG_SECTION		(63.6396)
#define DIAG_HALF_SECTION	(DIAG_SECTION/2.0)
#define SEARCH_HOSEI		(48.0)

#define TIRE_DIAMETER	(14.0f)							//mm
#define TIRE_RADIUS		(TIRE_DIAMETER/2.0f)			//mm
#define MMPP			(TIRE_DIAMETER*PI/ENC_RESOLUTION)	//mm
#define TREAD_WIDTH		((18.0-2.0)*2)

#ifdef __cplusplus
}
#endif


#endif /* MODULE_INC_MACRO_H_ */
