// SRexternal_device.h

#ifndef __SREXTERNAL_DEVICE_H__
#define __SREXTERNAL_DEVICE_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif

#define EXTERNAL_DEVICE_PORT		PORTD
#define EXTERNAL_DEVICE_PIN_bm		PIN4_bm


/* INITIALIZATION */

void external_device_init(void);

/* USER ACCESSIBLE FUNCTIONS */

void external_device_on(void);

void external_device_off(void);

uint8_t external_device_is_powered(void);
// returns boolean: 
//	0 = NO/OFF
//	1 = YES/ON


#ifdef __cplusplus
}
#endif

#endif