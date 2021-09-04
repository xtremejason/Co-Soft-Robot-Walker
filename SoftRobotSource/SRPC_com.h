// SRPC_com.h

#ifndef __SRPC_COM_H__
#define __SRPC_COM_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"
#include <stdio.h>

#include "../SoftRobotMain/xgrid_usart.h"

extern Usart usart;

// note: the standard parameter to pass to these is '&usart'

float type_a_number_blocking(Usart* usart);

float type_yes_or_no_blocking(Usart* usart);

#endif