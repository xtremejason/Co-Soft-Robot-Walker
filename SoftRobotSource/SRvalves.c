// SRvalves.c

#include <avr/io.h>
#include <stdio.h>

#include "SRvalves.h"


#define VALVE_DEBUG_LEVEL	0

/* FOR REFERENCE ONLY:
#define VALVE_1_bm 0b00000001
#define VALVE_2_bm 0b00000010
#define VALVE_3_bm 0b00000100
#define VALVE_4_bm 0b00001000
#define VALVE_5_bm 0b00010000
#define VALVE_6_bm 0b00100000
*/

#define MAX_VALVE_VENT_TIMESTEPS 100
#define MAX_VALVE_FILL_TIMESTEPS 100

// TODO: what is this used for??
uint8_t open_vent_valve_count[6];
uint8_t open_fill_valve_count[6];

void valves_init()
{
	// Pins as output
	VALVE_1_PORT.DIRSET = VALVE_11_PIN_bm | VALVE_12_PIN_bm;
	VALVE_2_PORT.DIRSET = VALVE_21_PIN_bm | VALVE_22_PIN_bm;
	VALVE_3_PORT.DIRSET = VALVE_31_PIN_bm | VALVE_32_PIN_bm;
	VALVE_4_PORT.DIRSET = VALVE_41_PIN_bm | VALVE_42_PIN_bm;
	VALVE_5_PORT.DIRSET = VALVE_51_PIN_bm | VALVE_52_PIN_bm;
	VALVE_6_PORT.DIRSET = VALVE_61_PIN_bm | VALVE_62_PIN_bm;

	for(uint8_t v = 0; v < 6; v++)
	{
		open_vent_valve_count[v] = 0;
		open_fill_valve_count[v] = 0;
	}
}

void valve_fill(uint8_t act_num)
{
	if(VALVE_DEBUG_LEVEL > 0)
		printf("fill %i\r\n",act_num);

	switch(act_num)
	{
		case 1:
			VALVE_1_PORT.OUTSET = VALVE_11_PIN_bm;
			VALVE_1_PORT.OUTCLR = VALVE_12_PIN_bm;
			break;
		case 2:
			VALVE_2_PORT.OUTSET = VALVE_21_PIN_bm;
			VALVE_2_PORT.OUTCLR = VALVE_22_PIN_bm;
			break;
		case 3:
			VALVE_3_PORT.OUTSET = VALVE_31_PIN_bm;
			VALVE_3_PORT.OUTCLR = VALVE_32_PIN_bm;
			break;
		case 4:
			VALVE_4_PORT.OUTSET = VALVE_41_PIN_bm;
			VALVE_4_PORT.OUTCLR = VALVE_42_PIN_bm;
			break;
		case 5:
			VALVE_5_PORT.OUTSET = VALVE_51_PIN_bm;
			VALVE_5_PORT.OUTCLR = VALVE_52_PIN_bm;
			break;
		case 6:
			VALVE_6_PORT.OUTSET = VALVE_61_PIN_bm;
			VALVE_6_PORT.OUTCLR = VALVE_62_PIN_bm;
			break;
	}

	open_vent_valve_count[act_num-1] = 0;
	open_fill_valve_count[act_num-1] = 1;
}

void valve_vent(uint8_t act_num)
{
	if(VALVE_DEBUG_LEVEL > 0)
		printf("vent %i\r\n",act_num);

	switch(act_num)
	{
		case 1:
			VALVE_1_PORT.OUTCLR = VALVE_11_PIN_bm;
			VALVE_1_PORT.OUTSET = VALVE_12_PIN_bm;
			break;
		case 2:
			VALVE_2_PORT.OUTCLR = VALVE_21_PIN_bm;
			VALVE_2_PORT.OUTSET = VALVE_22_PIN_bm;
			break;
		case 3:
			VALVE_3_PORT.OUTCLR = VALVE_31_PIN_bm;
			VALVE_3_PORT.OUTSET = VALVE_32_PIN_bm;
			break;
		case 4:
			VALVE_4_PORT.OUTCLR = VALVE_41_PIN_bm;
			VALVE_4_PORT.OUTSET = VALVE_42_PIN_bm;
			break;
		case 5:
			VALVE_5_PORT.OUTCLR = VALVE_51_PIN_bm;
			VALVE_5_PORT.OUTSET = VALVE_52_PIN_bm;
			break;
		case 6:
			VALVE_6_PORT.OUTCLR = VALVE_61_PIN_bm;
			VALVE_6_PORT.OUTSET = VALVE_62_PIN_bm;
			break;
	}

	open_vent_valve_count[act_num-1] = 1;
	open_fill_valve_count[act_num-1] = 0;
}

void valve_hold(uint8_t act_num)
{
	if(VALVE_DEBUG_LEVEL > 0)
		printf("hold %i\r\n",act_num);
	
	switch(act_num)
	{
		case 1:
			VALVE_1_PORT.OUTCLR = VALVE_11_PIN_bm;
			VALVE_1_PORT.OUTCLR = VALVE_12_PIN_bm;
			break;
		case 2:
			VALVE_2_PORT.OUTCLR = VALVE_21_PIN_bm;
			VALVE_2_PORT.OUTCLR = VALVE_22_PIN_bm;
			break;
		case 3:
			VALVE_3_PORT.OUTCLR = VALVE_31_PIN_bm;
			VALVE_3_PORT.OUTCLR = VALVE_32_PIN_bm;
			break;
		case 4:
			VALVE_4_PORT.OUTCLR = VALVE_41_PIN_bm;
			VALVE_4_PORT.OUTCLR = VALVE_42_PIN_bm;
			break;
		case 5:
			VALVE_5_PORT.OUTCLR = VALVE_51_PIN_bm;
			VALVE_5_PORT.OUTCLR = VALVE_52_PIN_bm;
			break;
		case 6:
			VALVE_6_PORT.OUTCLR = VALVE_61_PIN_bm;
			VALVE_6_PORT.OUTCLR = VALVE_62_PIN_bm;
			break;
	}

	open_vent_valve_count[act_num-1] = 0;
	open_fill_valve_count[act_num-1] = 0;
}

void valve_thru_open(uint8_t act_num)
{
	if(VALVE_DEBUG_LEVEL > 0)
		printf("vent %i\r\n",act_num);

	switch(act_num)
	{
		case 1:
		VALVE_1_PORT.OUTSET = VALVE_11_PIN_bm;
		VALVE_1_PORT.OUTSET = VALVE_12_PIN_bm;
		break;
		case 2:
		VALVE_2_PORT.OUTSET = VALVE_21_PIN_bm;
		VALVE_2_PORT.OUTSET = VALVE_22_PIN_bm;
		break;
		case 3:
		VALVE_3_PORT.OUTSET = VALVE_31_PIN_bm;
		VALVE_3_PORT.OUTSET = VALVE_32_PIN_bm;
		break;
		case 4:
		VALVE_4_PORT.OUTSET = VALVE_41_PIN_bm;
		VALVE_4_PORT.OUTSET = VALVE_42_PIN_bm;
		break;
		case 5:
		VALVE_5_PORT.OUTSET = VALVE_51_PIN_bm;
		VALVE_5_PORT.OUTSET = VALVE_52_PIN_bm;
		break;
		case 6:
		VALVE_6_PORT.OUTSET = VALVE_61_PIN_bm;
		VALVE_6_PORT.OUTSET = VALVE_62_PIN_bm;
		break;
	}

	open_vent_valve_count[act_num-1] = 1;
	open_fill_valve_count[act_num-1] = 1;
}

uint8_t fill_valve_is_open(uint8_t act_num)
{
	// act_num : 1-6

	if(open_fill_valve_count[act_num-1] > 0)
		return 1;
	else
		return 0;
}

uint8_t vent_valve_is_open(uint8_t act_num)
{
	// act_num : 1-6

	if(open_vent_valve_count[act_num-1] > 0)
		return 1;
	else
		return 0;
}


void valve_fill_ALL(void)
{
	for(uint8_t a = 1; a <= 6; a++)
		valve_fill(a);
	
	/*	
	valve_fill(1);
	valve_fill(2);
	valve_fill(3);
	valve_fill(4);
	valve_fill(5);
	valve_fill(6);
	*/
}

void valve_vent_ALL(void)
{
	for(uint8_t a = 1; a <= 6; a++)
		valve_vent(a);
	
	/*
	valve_vent(1);
	valve_vent(2);
	valve_vent(3);
	valve_vent(4);
	valve_vent(5);
	valve_vent(6);
	*/
}

void valve_hold_ALL(void)
{
	for(uint8_t a = 1; a <= 6; a++)
		valve_hold(a);
	
	/*
	valve_hold(1);
	valve_hold(2);
	valve_hold(3);
	valve_hold(4);
	valve_hold(5);
	valve_hold(6);
	*/
}

void valve_thru_open_ALL(void)
{
	for(uint8_t a = 1; a <= 6; a++)
		valve_thru_open(a);
}


/* INTERNAL FUNCTIONS */

void check_open_valves(void)
{
	for(uint8_t v = 0; v < 6; v++)
	{
		if(open_vent_valve_count[v] > 0)
		{// then we assume valve is venting
			open_vent_valve_count[v]++;

			if(open_vent_valve_count[v] > MAX_VALVE_VENT_TIMESTEPS)
			{// actuator should be empty by now, close the valve
				valve_hold(v+1);
			}
		}

		if(open_fill_valve_count[v] > 0)
		{// then we assume valve is venting
			open_fill_valve_count[v]++;

			if(open_fill_valve_count[v] > MAX_VALVE_VENT_TIMESTEPS)
			{// actuator should be full by now, close the valve
				valve_hold(v+1);
			}
		}
	}
}