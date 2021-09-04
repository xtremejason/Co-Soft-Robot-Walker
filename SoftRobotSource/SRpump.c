// SRpump.c

#include "SRpump.h"

#include <stdio.h>	// for printf


#define PUMP_PORT 			PORTD
#define PUMP_PIN_bm			PIN1_bm
#define PUMP_TC				TCD0			// on TCD0-CCB
#define PUMP_CCREG			CCB				// on Clock-Compare-B
#define PUMP_CC_ENABLE_bm	TC0_CCBEN_bm	// on Clock-Compare-B

uint8_t pump_speed;

#define PUMP_DEBUG_LEVEL	0		// none
//#define PUMP_DEBUG_LEVEL	1		// one


void pump_init()
{
	// Pins as output
	PUMP_PORT.DIRSET = PUMP_PIN_bm;
	
	//---------2-----------
	PUMP_TC.PER = 255;							// Set period (~500KHz -> abt. 500 ticks/ms)
	PUMP_TC.CCB = 0;							// initially fill the CCB register with a value
	PUMP_TC.CTRLA |= TC_CLKSEL_DIV1_gc;			// Set clock and prescaler, 32MHz/64 = 500KHz **
	PUMP_TC.CTRLB |= TC_WGMODE_SS_gc;			// enable Single Slope PWM (Waveform Generation Mode)
	PUMP_TC.CTRLB |= TC1_CCBEN_bm;				// enable waveform output on OCnA (setting WGM operation overrides the port output register for this output pin)

	pump_speed = 0;
}

uint8_t get_pump_power(void)
{
	return PUMP_TC.CCB;
}

void set_pump_power(uint8_t pow) 
{
	if(pow != pump_speed)
	{
		if(PUMP_DEBUG_LEVEL >= 1)
			printf("pump set: %u->%u\r\n", pump_speed, pow);
		
		PUMP_TC.CCBBUF = pow;	
		pump_speed = pow;
	}
}

uint8_t get_pump_is_on(void)
{
	//if(PUMP_TC.CCB >= PUMP_SPEED_MINIMUM)
	//if(PUMP_TC.CCB > PUMP_SPEED_OFF)
	if(pump_speed > 0)
		return 1;
	else
		return 0;
}

//////////////////////

/*
void pump_init_DEBUG()
{
	// Pins as output
	PUMP_PORT.DIRCLR = PUMP_PIN_bm;
}

void pump_input_check_DEBUG()
{
	uint8_t portdin = PUMP_PORT.IN;
	
	printf("PORTD: ");
	
	for(uint8_t i = 0; i < 8; i++)
	{
		printf("%u", (portdin >> (7-i)) & 1);
	}
	
	printf("\r\n");
}
*/	