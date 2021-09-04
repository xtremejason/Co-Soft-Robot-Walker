// SRpump.h

#ifndef __SRPUMP_H__
#define __SRPUMP_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif



// Pump speed (power) settings, all out of 255		
#define PUMP_SPEED_DEFAULT		200
#define PUMP_SPEED_HI			255
#define PUMP_SPEED_LOW			100
#define PUMP_SPEED_MINIMUM		 10		// lower than this, the pump doesn't turn
#define PUMP_SPEED_OFF			  0




void pump_init(void);

uint8_t get_pump_power(void);

void set_pump_power(uint8_t pow);

uint8_t get_pump_is_on(void);

float get_pump_drag_coef(void);


////////////////////////////////////

/*
void pump_init_DEBUG();

void pump_input_check_DEBUG();
*/

#ifdef __cplusplus
}
#endif

#endif