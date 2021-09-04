// SRstatus_LED.c

//#include <avr/io.h>
#include "SRstatus_LED.h"

/* INITIALIZATION */

// Set status LED pins as output & initialize timers for PWM
void status_LED_init()
{
	// LED Pins as output
	LED_1_PORT.DIRSET = LED_1_PIN_bm;
	LED_2_PORT.DIRSET = LED_2_PIN_bm;
	
	//---------1-----------
	LED_1_TC.PER = 255;							// Set period (~500KHz -> abt. 500 ticks/ms) 
	//LED_1_TC.CCB = 0;							// initially fill the CCB register with a value
	LED_1_TC.LED_1_CCREG = 0;					// initially fill the CCB register with a value
	LED_1_TC.CTRLA |= TC_CLKSEL_DIV1_gc;		// Set clock and prescaler, 32MHz/64 = 500KHz ** 
	LED_1_TC.CTRLB |= TC_WGMODE_SS_gc;			// enable Single Slope PWM (Waveform Generation Mode)
	//LED_1_TC.CTRLB |= TC1_CCBEN_bm;			// enable waveform output on OCnB (setting WGM operation overrides the port output register for this output pin)
	LED_1_TC.CTRLB |= LED_1_CC_ENABLE_bm;		// enable waveform output on OCnB (setting WGM operation overrides the port output register for this output pin)
	

	//---------2-----------
	LED_2_TC.PER = 255;							// Set period (~500KHz -> abt. 500 ticks/ms) 
	//LED_2_TC.CCB = 0;							// initially fill the CCB register with a value
	LED_2_TC.LED_2_CCREG = 0;					// initially fill the CCA register with a value
	LED_2_TC.CTRLA |= TC_CLKSEL_DIV1_gc;		// Set clock and prescaler, 32MHz/64 = 500KHz ** 
	LED_2_TC.CTRLB |= TC_WGMODE_SS_gc;			// enable Single Slope PWM (Waveform Generation Mode)
	//LED_2_TC.CTRLB |= TC1_CCBEN_bm;			// enable waveform output on OCnA (setting WGM operation overrides the port output register for this output pin)
	LED_2_TC.CTRLB |= LED_2_CC_ENABLE_bm;		// enable waveform output on OCnA (setting WGM operation overrides the port output register for this output pin)
}

void toggle_LED(uint8_t LED_num)
{
	if(get_LED(LED_num))
		set_LED(LED_num, 0);
	else
		set_LED(LED_num, LED_SATURATION_NORMAL);
}

uint8_t get_LED(uint8_t LED_num)
{
	if(LED_num == 1)
		return LED_1_TC.LED_1_CCREG;

	else if(LED_num == 2)
		return LED_2_TC.LED_2_CCREG;
		
	else
		printf("ERROR: no LED %u\r\n", LED_num);

	return 0;
}

void set_LED(uint8_t LED_num, uint8_t saturation)
{
	// DEBUG:
	//printf("LED %u: %u\r\n", LED_num, saturation);
	
	if(LED_num == 1)
		LED_1_TC.LED_1_CCBUF = saturation;	
	else if(LED_num == 2)
		LED_2_TC.LED_2_CCBUF = saturation;
	else
		printf("ERROR: no LED %u\r\n", LED_num);
}
	