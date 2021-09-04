// SRhardware_calibration.c

#include "SRhardware_calibration.h"

#include <util/delay.h>

// Soft Robot PCB Identity:
//#define MAIN_PCB_ID		'1'		// black button (chip ID: af16)
#define MAIN_PCB_ID		'2'		// white button (chip ID: 9313)

#define NO_PCB				0	// constant
// Actuator PCB placement in the 6 PCB slots on the main board:

	// 'white button robot'
#define ACTUATOR_1_PCB_ID	'A'
#define ACTUATOR_2_PCB_ID	NO_PCB
#define ACTUATOR_3_PCB_ID	'B'
#define ACTUATOR_4_PCB_ID	'C'
#define ACTUATOR_5_PCB_ID	NO_PCB
#define ACTUATOR_6_PCB_ID	'E'

/*	// 'black button robot'
#define ACTUATOR_1_PCB_ID	'D'
#define ACTUATOR_2_PCB_ID	NO_PCB
#define ACTUATOR_3_PCB_ID	NO_PCB
#define ACTUATOR_4_PCB_ID	NO_PCB
#define ACTUATOR_5_PCB_ID	NO_PCB
#define ACTUATOR_6_PCB_ID	NO_PCB
*/

/* Analog pressure sensor calibration (ADC to PSI equation) */ 
// Actuator PCB 'A'
#define ADC_PSI_A1_YINT		-0.281159
#define ADC_PSI_A1_SLOPE	0.040346
#define ADC_PSI_A2_YINT		-0.236334
#define ADC_PSI_A2_SLOPE	0.040573

// Actuator PCB 'B'
#define ADC_PSI_B1_YINT		-0.159117
#define ADC_PSI_B1_SLOPE	0.040954
#define ADC_PSI_B2_YINT		-0.123818
#define ADC_PSI_B2_SLOPE	0.041151

// Actuator PCB 'C'
#define ADC_PSI_C1_YINT		-0.463289
#define ADC_PSI_C1_SLOPE	0.040249
#define ADC_PSI_C2_YINT		-0.392845
#define ADC_PSI_C2_SLOPE	0.040096

// Actuator PCB 'D'
#define ADC_PSI_D1_YINT		-0.349570
#define ADC_PSI_D1_SLOPE	0.040049
#define ADC_PSI_D2_YINT		-0.352999
#define ADC_PSI_D2_SLOPE	0.040560

// Actuator PCB 'E'
#define ADC_PSI_E1_YINT		-0.32	// needs calibrated
#define ADC_PSI_E1_SLOPE	0.038
#define ADC_PSI_E2_YINT		-0.314081	// 4/24/16	(act 3 position, V4-5)
#define ADC_PSI_E2_SLOPE	0.038398


#define PUMP_1_SMALL		1
#define PUMP_4_ROUND		4

//#define PUMP_ID		PUMP_1_SMALL
#define PUMP_ID		PUMP_4_ROUND


// this is how much the ADC values are dragged down by the pump running
float pump_ADC_drag_coef = 1.0;	// less than 1.0 if voltage drops
// set here just in case it is not initialized (we divide by this, so it shouldn't be 0)

// note that these could entirely depend upon the PC used to supply the power, or even the USB port within the PC (untested!)
#define PUMP_DRAG_COEF_ROUND	0.9
#define PUMP_DRAG_COEF_SMALL	0.97	// meas 4/24/16

#define NO_PUMP_NO_VALVE_COEF	1.02	// meas 4/24/16, or 1.023

uint8_t hardware_calibration_main_PCB_ID(void)
{
	return MAIN_PCB_ID;
}


uint8_t hardware_calibration_actuator_PCB_ID(uint8_t act_num)
{
	uint8_t PCB_ID;
	
	switch(act_num)
	{
		case 1:	PCB_ID = ACTUATOR_1_PCB_ID;	break;
		case 2: PCB_ID = ACTUATOR_2_PCB_ID;	break;
		case 3:	PCB_ID = ACTUATOR_3_PCB_ID;	break;
		case 4: PCB_ID = ACTUATOR_4_PCB_ID;	break;
		case 5:	PCB_ID = ACTUATOR_5_PCB_ID;	break;
		case 6: PCB_ID = ACTUATOR_6_PCB_ID;	break;
		default:
			printf("ERROR: invalid ");
			_delay_ms(5);
			printf("actuator: %u\r\n", act_num);
			PCB_ID = NO_PCB;
			break;
	}
		
	return PCB_ID;	
}

void hardware_calibration_ADCtoPSI_values(uint8_t actuator_PCB_ID, float *slope, float *yint)
{
	
	if((MAIN_PCB_ID != '1')&&(MAIN_PCB_ID != '2'))
	{
		// you shouldn't be able to get here!
		printf("ERROR: unknown ");
		_delay_ms(5);
		printf("MAIN PCB: '%c'\r\n", MAIN_PCB_ID);
		_delay_ms(5);
		
		*yint = 0;
		*slope = 0;
		return;	// error
	}
		
	switch(actuator_PCB_ID)
	{
		case 'A':
			if(MAIN_PCB_ID == '1'){
				*yint = ADC_PSI_A1_YINT;
				*slope = ADC_PSI_A1_SLOPE;}
			else if(MAIN_PCB_ID == '2'){
				*yint = ADC_PSI_A2_YINT;
				*slope = ADC_PSI_A2_SLOPE;}
			break;
		case 'B':
			if(MAIN_PCB_ID == '1'){
				*yint = ADC_PSI_B1_YINT;
				*slope = ADC_PSI_B1_SLOPE;}
			else if(MAIN_PCB_ID == '2'){
				*yint = ADC_PSI_B2_YINT;
				*slope = ADC_PSI_B2_SLOPE;}
			break;
		case 'C':
			if(MAIN_PCB_ID == '1'){
				*yint = ADC_PSI_C1_YINT;
				*slope = ADC_PSI_C1_SLOPE;}
			else if(MAIN_PCB_ID == '2'){
				*yint = ADC_PSI_C2_YINT;
				*slope = ADC_PSI_C2_SLOPE;}
			break;
		case 'D':
			if(MAIN_PCB_ID == '1'){
				*yint = ADC_PSI_D1_YINT;
				*slope = ADC_PSI_D1_SLOPE;}
			else if(MAIN_PCB_ID == '2'){
				*yint = ADC_PSI_A2_YINT;
				*slope = ADC_PSI_A2_SLOPE;}
			break;
		case 'E':
			if(MAIN_PCB_ID == '1'){
				*yint = ADC_PSI_E1_YINT;
				*slope = ADC_PSI_E1_SLOPE;}
			else if(MAIN_PCB_ID == '2'){
				*yint = ADC_PSI_E2_YINT;
				*slope = ADC_PSI_E2_SLOPE;}
			break;
		default:
			printf("ERROR: invalid ");
			_delay_ms(5);
			printf("actuator ID: '%c'\r\n", actuator_PCB_ID);
			*yint = 0;
			*slope = 0;
			break;
	}
	
	return;
}

void set_pump_ADC_drag_coef(void)
{
	if(PUMP_ID == PUMP_4_ROUND)
		pump_ADC_drag_coef == PUMP_DRAG_COEF_ROUND;
	else if(PUMP_ID == PUMP_1_SMALL)
		pump_ADC_drag_coef == PUMP_DRAG_COEF_SMALL;
	else
		pump_ADC_drag_coef = 1.0;
}

float pressure_drag_drop_correction(float p_meas, uint8_t act_num)
{
	float pressure;
	
	if(get_pump_is_on())	// if pump is on, we assume that dominates the voltage drop (TODO: verify)
	{
		pressure = p_meas/pump_ADC_drag_coef;
		if(CALIBRATION_DEBUG_PRINT)	printf("!");
	}
	else if( (!fill_valve_is_open(act_num)) && (!vent_valve_is_open(act_num)) ) // no pump or any valve is on
	{
		pressure = p_meas/(float)NO_PUMP_NO_VALVE_COEF;
		if(CALIBRATION_DEBUG_PRINT)	printf("?");
	}
	else
	{
		pressure = p_meas;
		//printf("[%.3f -> %.3f]", p_meas, pressure);
		if(CALIBRATION_DEBUG_PRINT)	printf("*");
	}
	
	return pressure;
}


void load_default_ADCtoPSI_calibration(void)
{
	printf("Default PCB arrangement:\r\n");
	_delay_ms(10);
	
	float slope, yint;
	uint8_t main_PCB_ID;		// char
	uint8_t acutuator_PCB_ID;	// char
	
	main_PCB_ID = hardware_calibration_main_PCB_ID();
	
	set_pump_ADC_drag_coef();
	
	for(uint8_t act_num = 1; act_num <= 6; act_num++)
	{
		acutuator_PCB_ID = hardware_calibration_actuator_PCB_ID(act_num);
		
		if(acutuator_PCB_ID != NO_PCB)
		{
			// DEBUG print:
			printf("Act#%u|'%c'\r\n", act_num, acutuator_PCB_ID);
			_delay_ms(5);
			
			// load the slope and y-intercept as a pair, by reference
			hardware_calibration_ADCtoPSI_values(acutuator_PCB_ID, &slope, &yint);
			
			save_new_ADCtoPSI_calibration(act_num, slope, yint);
		}
	}
	
	return;
}

void clear_all_ADCtoPSI_calibration(void)
{
	printf("clearing all PCB arrangement\r\n");
	_delay_ms(10);
	
	for(uint8_t a = 1; a <= 6; a++)
	{
		save_new_ADCtoPSI_calibration(a, 0, 0);
		set_ADC_pressure_calibrated(a, 0);
	}
	
	return;
}
