/*
 * communicate.h
 *
 *  Created on: 2024/03/08
 *      Author: sato1
 */

#ifndef CPP_MODULE_INC_COMMUNICATE_H_
#define CPP_MODULE_INC_COMMUNICATE_H_

#include <stdio.h>
#include "main.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

	uint8_t 	Communicate_TerminalRecv( void );		// 1文字受信
	void 		Communicate_Initialize( void );			// printfとscanfを使用するための設定
	void Communicate_TxPopData( void );
	void Communicate_TxPushData( int8_t data );
	uint8_t Communicate_RxPopData( void );
	void Communicate_RxPushData( void );
	uint8_t Communicate_RecieveDMA( void );


#ifdef __cplusplus
}
#endif



#endif /* CPP_MODULE_INC_COMMUNICATE_H_ */
