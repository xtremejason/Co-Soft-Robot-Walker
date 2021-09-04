//SRtouch_control.c

#include "SRtouch_control.h"

#include <util/delay.h>		// for printing delays only
#include <stdio.h>

#include "../SoftRobotSource/SRcapacitive_touch.h"
#include "../SoftRobotSource/SRpump.h"
#include "../SoftRobotSource/SRmisc_functions.h"
#include "../SoftRobotSource/SRanalog_pressure.h"
#include "../SoftRobotSource/SRvalves.h"

#include "SRpressure_control.h"


#define TOUCH_CONTROL_DEBUG  	0	// no printing
//#define TOUCH_CONTROL_DEBUG  	1	// print touch/untouch updates


// arrays for storing the 1:1 mapping of pads to electrodes
uint8_t pad_electrode[MAX_NUM_TOUCH_PADS];
uint8_t num_touch_pads = MAX_NUM_TOUCH_PADS;	// proximal to distal ordering, sensor strip specific

float pad_pos[MAX_NUM_TOUCH_PADS];



// private variables

uint8_t touch_controller_enabled;

uint8_t previous_touch; //previous iteration from get_touch_status_change(), initialized in init_touch
uint8_t previous_untouch; //previous iteration from get_untouch_status_change(), initialized in init_touch

/* PAD-LEVEL FUNCTIONS */

uint8_t get_cap_values_allpads(float *values, uint8_t type, uint8_t baseline)
{
	//uint16_t e_vals[NUM_ELECTRODES];
	uint16_t e_vals[MPR121_NUM_ELECTRODES];
	uint8_t p_i;	// pad index

	// (TODO) shrink these array sizes from the default (12) to the actual max used (8)
	//	currently this is not possible if the legacy (no-PCB-prototype) sensor strips are used
	if(baseline)	get_electrodes_baseline(e_vals, MPR121_NUM_ELECTRODES);
	else			get_electrodes_values(e_vals, MPR121_NUM_ELECTRODES);
	
	for(p_i = 0; p_i < num_touch_pads; p_i++)
	{
		if(type == CAP_VALUE_TYPE_ADC)
		{
			values[p_i] = (float)e_vals[pad_electrode[p_i]];
		}

		else if(type == CAP_VALUE_TYPE_CAPACITANCE)
		{
			values[p_i] = C_ADC_to_capacitance(e_vals[pad_electrode[p_i]]);
		}
	}

	return num_touch_pads;
}

void print_cap_values_allpads(uint8_t type, uint8_t baseline)
{
	uint16_t e_vals[MPR121_NUM_ELECTRODES];
	uint16_t b_vals[MPR121_NUM_ELECTRODES];
	float pad_val;
	uint8_t p_i;	// pad index

	// (TODO) this function has a lot of overlap with the 'get' function, merge these duplicate lines
	if(baseline)	get_electrodes_baseline(e_vals, MPR121_NUM_ELECTRODES);
	else			get_electrodes_values(e_vals, MPR121_NUM_ELECTRODES);

	get_electrodes_baseline(b_vals, MPR121_NUM_ELECTRODES);
	
	for(p_i = 0; p_i < num_touch_pads; p_i++)
	{
		pad_val = reshape_C_ADC(e_vals[pad_electrode[p_i]], b_vals[pad_electrode[p_i]], type);
		
		printf("%.0f", pad_val);
		if(p_i+1 < num_touch_pads)	printf(", ");
		else						printf("\r\n");
		_delay_ms(2);
	}
}

float get_cap_value_1pad(uint8_t touch_pad_number, uint8_t type, uint8_t baseline)
{
	float pad_vals[MAX_NUM_TOUCH_PADS];

	get_cap_values_allpads(pad_vals, type, baseline);

	return pad_vals[touch_pad_number - 1];
}


/* ACTUATOR-LEVEL FUNCTIONS */

uint8_t get_touch_status_change(uint8_t *touch_change, uint8_t *untouch_change)
{
	uint8_t current_touch;

	//if(verify_touch_existance())
	if(touch_controller_enabled && verify_touch_existance())
	{
		current_touch = get_touch_pads_in_contact();
		
		*touch_change = 0;
		*untouch_change = 0;

		for (uint8_t pad_num = 1; pad_num <= num_touch_pads; pad_num++)
		{
			if((flag_is_on(current_touch, pad_num))&&(!flag_is_on(previous_touch, pad_num)))
			{
				*touch_change = make_flag(*touch_change, pad_num);
			}

			if((!flag_is_on(current_touch, pad_num))&&(flag_is_on(previous_untouch, pad_num)))
			{
				*untouch_change = make_flag(*untouch_change, pad_num);
			}
		}

		previous_touch = current_touch;
		previous_untouch = current_touch;
	}

	if(*touch_change && TOUCH_CONTROL_DEBUG)
	{
		printf("touch:");
		print_uint8_in_binary(*touch_change);
	}

	if(*untouch_change && TOUCH_CONTROL_DEBUG)
	{
		printf("untouch:");
		print_uint8_in_binary(*untouch_change);
	}

	return (*touch_change|*untouch_change);
}

uint8_t get_touch_pads_in_contact(void)
{
	//uint16_t cap_value[NUM_TOUCH_PADS];
	//uint16_t base_value[NUM_TOUCH_PADS];
	float cap_value[num_touch_pads];
	float base_value[num_touch_pads];
	uint8_t flag = 0;
	float percentdiff = 0;

	//get_C_ADC_values_all_pads(cap_value);
	//get_baseline_C_ADC_values_all_pads(base_value);
	get_cap_values_allpads(base_value, CAP_VALUE_TYPE_ADC, 1);
	get_cap_values_allpads(cap_value, CAP_VALUE_TYPE_ADC, 0);
	
	for (uint8_t i = 0; i < num_touch_pads; i++)
	{
		percentdiff = (base_value[i] - cap_value[i])/(base_value[i]) * 100.0;

		if(percentdiff > CONTACT_THRESHOLD_PCT)
		{
			flag = make_flag(flag, i+1);
		}
	}
	return flag;
}

uint8_t get_num_touch_pads(void)
{
	return num_touch_pads;
}


/* MAIN-LOOP FUNCTION (CONTROLLER) */

void touch_controller(uint8_t touch_events, uint8_t untouch_events)
{	
	float start_pressure, final_pressure;
	
	if(touch_controller_enabled && touch_events)
	{
		start_pressure = get_actuator_pressure_PSI(TOUCH_ACTUATOR_NUM);
		printf("{START: %f}\r\n", start_pressure);

		if(flag_is_on(touch_events, FILL_TOUCHPAD))		// push cap-touch button
		{	
			valve_fill(TOUCH_ACTUATOR_NUM);
			set_pump_speed_manual(PUMP_SPEED_HI);
		}

		if(flag_is_on(touch_events, VENT_TOUCHPAD))		// push cap-touch button
		{	
			valve_vent(TOUCH_ACTUATOR_NUM);
		}
	}

	if(touch_controller_enabled && untouch_events)
	{
		if(flag_is_on(untouch_events, FILL_TOUCHPAD))	// release cap-touch button
		{	
			valve_hold(TOUCH_ACTUATOR_NUM);
			set_pump_speed_manual(0);
		}

		if(flag_is_on(untouch_events, VENT_TOUCHPAD))	// release cap-touch button
		{	
			valve_hold(TOUCH_ACTUATOR_NUM);
		}

		final_pressure = get_actuator_pressure_PSI(TOUCH_ACTUATOR_NUM);
		printf("{STOP: %f}\r\n", final_pressure);
	}
}


/* INITIALIZATION */

void init_touch_control(void)
{
	if(check_touch_exists())
	{
		//touch_controller_enabled = 0;
		touch_controller_enabled = TOUCH_CONTROL_ON;

		previous_touch = 0;
		previous_untouch = 0;

		define_electrode_strip_connections(SENSOR_STRIP_VERSION);

		if(SENSOR_STRIP_VERSION >= 3)
			enable_MPR121_individual_electrodes_bits(8);	// switch from all 12 -> first 8

		if(touch_controller_enabled)
			printf(" TC-ON\r\n");
		else
			printf(" TC-OFF\r\n");

	}

	else
		printf(" TC-NONE\r\n");
	
}

void define_electrode_strip_connections(uint8_t strip_number)
{
	uint8_t i;
	
	// Pad to Electrode physical wire connections (hardware specific)
	// typical ordering, always given as proximal to distal ordering
	// squares, rectangles: {1,2,3,4,5,6,7,8} (strip 6, squares, is reversed)
	// triangles: {2,4,6,8,7,5,3,1} (strip 5, triangles, is reversed)
	num_touch_pads = 8;

	switch(strip_number)
	{
		case 1:		// square pads
		case 2:		// rectangle pads
		case 3:		// rectangle pads
			for(i = 0; i < num_touch_pads; i++)
			{
				pad_electrode[i] = i;
				pad_pos[i] = 1 + 2*i;
			}
			break;

		case 4:		// triangles pads
			pad_electrode[0] = 1;	pad_pos[0] = 2.2;
			pad_electrode[1] = 3;	pad_pos[1] = 3.98;	// delta = 1.78 each from 2.2
			pad_electrode[2] = 5;	pad_pos[2] = 5.76;
			pad_electrode[3] = 7;	pad_pos[3] = 7.54;
			pad_electrode[4] = 6;	pad_pos[4] = 9.32;
			pad_electrode[5] = 4;	pad_pos[5] = 11.1;
			pad_electrode[6] = 2;	pad_pos[6] = 13.65;	// delta = 2.55 from 11.1
			pad_electrode[7] = 0;	pad_pos[7] = 14.65;	// delta = 35.5 from 11.1
			break;
		
		case 5:		// triangles pads
			pad_electrode[0] = 0;	pad_pos[0] = 2.2;
			pad_electrode[1] = 2;	pad_pos[1] = 3.98;	// delta = 1.78 each from 2.2
			pad_electrode[2] = 4;	pad_pos[2] = 5.76;
			pad_electrode[3] = 6;	pad_pos[3] = 7.54;
			pad_electrode[4] = 7;	pad_pos[4] = 9.32;
			pad_electrode[5] = 5;	pad_pos[5] = 11.1;
			pad_electrode[6] = 3;	pad_pos[6] = 13.65;	// delta = 2.55 from 11.1
			pad_electrode[7] = 1;	pad_pos[7] = 14.65;	// delta = 35.5 from 11.1
			break;
		
		case 6:		// square pads
			for(i = 0; i < num_touch_pads; i++)
			{
				pad_electrode[i] = 7-i;	// reverse ordering
				pad_pos[i] = 1 + 2*i;
			}
			break;

		default:	
			for(uint8_t i = 0; i < num_touch_pads; i++)
			{
				pad_electrode[i] = i;
				pad_pos[i] = 1 + 2*i;
			}

			break;
	}
}


/* INTERNAL FUNCTIONS */

float reshape_C_ADC(uint16_t ADC_value, uint16_t baseline_value, uint8_t type)
{
	float val;
	float ret_val;

	switch(type)
	{
		case CAP_VALUE_TYPE_PROXIMITY:
			val = (baseline_value + 1.0 - ADC_value)/(float)baseline_value;
			ret_val = (-1)*log10(val);
			break;

		case CAP_VALUE_TYPE_ADC:
			ret_val = (float)ADC_value;
			break;

		case CAP_VALUE_TYPE_CAPACITANCE:
			ret_val = C_ADC_to_capacitance(ADC_value);
			break;

		case CAP_VALUE_TYPE_DELTA_ADC:
			ret_val = (float)baseline_value - (float)ADC_value;
			break;

		default:
			ret_val = 0;
			break;
	}

	return ret_val;
}


