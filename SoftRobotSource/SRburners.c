// SRburners.c

#include <avr/io.h>
#include "SRburners.h"

void burners_init(void)
{
	// Pins as output
	BURNER_PORT.DIRSET = BURNER1_PIN_bm;
	BURNER_PORT.DIRSET = BURNER2_PIN_bm;
	
}


void burner_on(uint8_t burner_num)
{
	switch(burner_num)
	{
		case 1:
			BURNER_PORT.OUTSET |= BURNER1_PIN_bm;
			break;
		case 2:
			BURNER_PORT.OUTSET |= BURNER2_PIN_bm;
			break;
		default:
			//TODO, there is no other option
			break;
	}	
}

void burner_off(uint8_t burner_num)
{
	switch(burner_num)
	{
		case 1:
			BURNER_PORT.OUTSET &= ~BURNER1_PIN_bm;
			break;
		case 2:
			BURNER_PORT.OUTSET &= ~BURNER2_PIN_bm;
			break;
		default:
			//TODO, there is no other option
			break;
	}
}

	