// SRanalog_pressure.c

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
//#include "../SoftRobotMain/xgrid_maintenance.h"	// required to use "SP_ReadCalibrationByte"
//#include "../xboot/sp_driver.h"
//#include "stddef.h"				// required to use "offsetof(_,_)" for production row access

#include "SRanalog_pressure.h"

#include "software_clock.h"	// for overtime permission
#include "SRhardware_calibration.h"
#include "SRpump.h"
#include "SRmisc_functions.h"

#define APS_ADC_DEBUG_LEVEL 0


/* ANALOG TO DIGITAL CONVERSION SETTTINGS */
// ADC INPUT PINS - PORT CONNECTIONS: 
#define PRESSURE1_PORT 			PORTA
#define PRESSURE1_PIN_bm		PIN3_bm
#define PRESSURE2_PORT 			PORTB
#define PRESSURE2_PIN_bm		PIN2_bm
#define PRESSURE3_PORT 			PORTB
#define PRESSURE3_PIN_bm		PIN1_bm
#define PRESSURE4_PORT 			PORTA
#define PRESSURE4_PIN_bm		PIN6_bm
#define PRESSURE5_PORT 			PORTA
#define PRESSURE5_PIN_bm		PIN0_bm
#define PRESSURE6_PORT 			PORTA
#define PRESSURE6_PIN_bm		PIN4_bm
// ADC INPUT PINS - ADCA/ADCB CONNECTIONS:
#define APS1_ADC		ADCA		// APS 1 on PA3
#define APS2_ADC		ADCB		// APS 2 on PB2
#define APS3_ADC		ADCB		// APS 3 on PB1
#define APS4_ADC		ADCA		// APS 4 on PA6
#define APS5_ADC		ADCA		// APS 5 on PA0
#define APS6_ADC		ADCA		// APS 6 on PA4
// ADC INPUT PINS - MUX POSITIONS:
#define APS1_MUXPOS_gc		ADC_CH_MUXPOS_PIN3_gc		// APS 1 on PA3
#define APS2_MUXPOS_gc		ADC_CH_MUXPOS_PIN2_gc		// APS 2 on PB2
#define APS3_MUXPOS_gc		ADC_CH_MUXPOS_PIN1_gc		// APS 3 on PB1
#define APS4_MUXPOS_gc		ADC_CH_MUXPOS_PIN6_gc		// APS 4 on PA6
#define APS5_MUXPOS_gc		ADC_CH_MUXPOS_PIN0_gc		// APS 5 on PA0
#define APS6_MUXPOS_gc		ADC_CH_MUXPOS_PIN4_gc		// APS 6 on PA4

// settings for self-diagnosis check:
#define DRIFT_CHECK_NUM_ITERATIONS 20  // this should remain 8 or above to give meaningful results for oscillating floating

// private variables:
float calibration_ADCtoPSI_yint[6];		// stores the y-intercept of the ADC->PSI conversion function (line)
float calibration_ADCtoPSI_slope[6];	// stores the slope of the ADC->PSI conversion function (line)

uint8_t ADC_pressure_calib_flags = 0;	// bit flags 0b00xxxxxx, stores which of the ADCs have been calibrated (have known ACD->PSI conversion function)


#define APS_DATA_EXPIRATION_TIME_ms	20	// NEW: to prevent repeat measurements of the same data
// in theory, this should be only 8 ms @ 125 Hz

uint16_t last_APS_ADC_meas_time_ms[6];
uint8_t last_APS_ADC_measurement[6];

uint8_t APS_ADC_lockout;

#define ADC_LOWEST_READING 11

uint8_t ADC_zero_reading[6];	// this value will typically be about 11 if using Vcc/1.6 reference (unsigned)
								// this is effectively the 'DELTA'V that is used in the math of the ADC in the manual

uint8_t APS_init_complete = 0;

void APS_init()
{
	// Analog Pressure Sensors use ADCA/ADCB channel 0, all the time

	/* (1) PORT CONFIGURATION */
	// SET INPUT PINS AS INPUTS 
	
	PRESSURE1_PORT.DIRCLR = PRESSURE1_PIN_bm;
	PRESSURE2_PORT.DIRCLR = PRESSURE2_PIN_bm;
	PRESSURE3_PORT.DIRCLR = PRESSURE3_PIN_bm;
	PRESSURE4_PORT.DIRCLR = PRESSURE4_PIN_bm;
	PRESSURE5_PORT.DIRCLR = PRESSURE5_PIN_bm;
	PRESSURE6_PORT.DIRCLR = PRESSURE6_PIN_bm;

	/* (2) ADC CONFIGURATION */

	// 28.16.3 REFCTRL – Reference Control register
	//
	// Bit 1 – BANDGAP: Bandgap enable
	// Setting this bit enables the Bandgap for ADC measurement. Note that if any other functions are
	// using the Bandgap already, this bit does not need to be set when the internal 1.00V reference is
	// used in ADC or DAC, or if the Brown-out Detector is enabled.
	//
	// Bits 6:4 – REFSEL[2:0]: ADC Reference Selection
	// These bits selects the reference for the ADC

	APS1_ADC.REFCTRL = ADC_REFSEL_VCC_gc;							// Vcc/1.6
	APS2_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	APS3_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	APS4_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	APS5_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	APS6_ADC.REFCTRL = ADC_REFSEL_VCC_gc;

	//APS3_ADC.REFCTRL = 0b01000000;								// Vcc/2
	//ADCB.REFCTRL = 0b00100000;									// AREFA = not provided
	//ADCB.REFCTRL = 0b00110000;									// AREFB = not provided

	// 28.16.2 CTRLB – ADC Control Register B
	//
	// Bit 7 – IMPMODE: Gain Stage Impedance Mode
	// This bit controls the impedance mode of the gain stage. See GAIN setting with ADC Channel
	// Register description for more information.
	//
	// Bit 6:5 – CURRLIMIT[1:0]: Current Limitation
	// These bits can be used to limit the maximum current consumption of the ADC. Setting these bits
	// will also reduce the maximum sampling rate. The available settings is shown in Table 28-3 on
	// page 367. The indicated current limitations are nominal values, refer to device datasheet for
	// actual current limitation for each setting.
	//
	// Bit 4 – CONVMODE: ADC Conversion Mode
	// This bit controls whether the ADC will work in signed or unsigned mode. By default this bit is
	// cleared and the ADC is configured for unsigned mode. When this bit is set the ADC is configured
	// for signed mode.
	//
	// Bit 3 – FREERUN: ADC Free Running Mode
	// When the bit is set to one, the ADC is in free running mode and ADC channels defined in the
	// EVCTRL register are swept repeatedly.
	//
	// Bit 2:1 – RESOLUTION[1:0]: ADC Conversion Result Resolution
	// These bits define whether the ADC completes the conversion at 12- or 8-bit result. They also
	// define whether the 12-bit result is left or right oriented in the 16-bit result registers. See Table
	// 28-4 on page 367 for possible settings.
	
	APS1_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;		// use 8 bit resolution
	APS2_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	APS3_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	APS4_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	APS5_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	APS6_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
			
			/* TROUBLES: currently, we cannot use the 12-bit resolution setting because we cannot
			read the TEMP register of the ADC, because the compiler does not recognize the TEMP
			register, because iox128a3u.h is NOT being used, rather iox128a3.h is being used.
			It is our makefile that governs this choice, because the makefile also is used for
			programming "make program" (links with avrdude) and avrdude does not support A3U,
			moreover WINAVR 'avr\include\avr\' folder's io.h does not recognize __AVR_ATxmega128A3U__
			
			??? I don't know where in the compile pipeline the makefile line:

			MCU = atxmega128a3u
			
						gets converted to the command in <io.h>:

			#elif defined (__AVR_ATxmega128A3U__)
			#  include <avr/iox128a3u.h>


			see: www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=120482&start for possibly
			helpful discussion
			??? ------------------------------------------------------------------------------ */

	// 28.16.5 PRESCALER – Clock Prescaler Register
	//
	// Bit 2:0 – PRESCALER[2:0]: ADC Prescaler configuration
	// These bits define the ADC clock relative to the Peripheral clock, according to Table 28-9 on
	// page 370.

	APS1_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	APS2_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	APS3_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	APS4_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	APS5_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	APS6_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;

	/* (3) ADC CHANNEL CONFIGURATION */
	
	APS1_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	APS2_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	APS3_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	APS4_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	APS5_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	APS6_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	/* (4) READ & LOAD CALIBRATION (PRODUCTION SIGNATURE ROW) */	//TODO (unlikely will make any difference)
	
	/*
	ADCBcalibration0 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCBCAL0 ));
	ADCBcalibration1 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCBCAL1 ));
	ADCAcalibration0 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCACAL0 ));
	ADCAcalibration1 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCACAL1 ));
	*/

	// SHOULD WE WRITE THEM???
	
	//ADCB.CALL = ADCBcalibration0;
	//ADCB.CALH = ADCBcalibration1;
	
	/* (5) ENABLE THE ADC SO THAT IT MAY MAKE MEASUREMENTS */

	APS1_ADC.CTRLA = ADC_ENABLE_bm;
	APS2_ADC.CTRLA = ADC_ENABLE_bm;
	APS3_ADC.CTRLA = ADC_ENABLE_bm;
	APS4_ADC.CTRLA = ADC_ENABLE_bm;
	APS5_ADC.CTRLA = ADC_ENABLE_bm;
	APS6_ADC.CTRLA = ADC_ENABLE_bm;

	/* (6) FIND AND RECORD THE ZERO-OFFSET OF EACH PRESSURE SENSOR (optional) */

	//get_APS_zeros(ADC_zero_reading);
	
	set_acutator_pressure_meas_lock();

	set_pump_ADC_drag_coef();

	APS_init_complete = 1;
}


void set_acutator_pressure_meas_lock(void)
{
	APS_ADC_lockout = 1;
}

void release_acutator_pressure_meas_lock(void)
{
	APS_ADC_lockout = 0;
}

uint8_t get_actuator_pressure_ADC(uint8_t actuator_num)
{	// this routine uses ADCA/ADCB channel 0, all the time
	// this routine takes 3 consecutive measurements, and only returns the median measurement!

	if(APS_ADC_DEBUG_LEVEL > 0)	printf("<%u", actuator_num);	// DEBUG
	
	if(APS_ADC_lockout == 1)
	{
		if(APS_ADC_DEBUG_LEVEL > 0)	printf("L>");
		return get_actuator_pressure_last_ADC(actuator_num);
	}
	
	uint8_t act_ind = actuator_num - 1;
	
	uint16_t current_time;
		
	uint8_t meas[3];
	uint16_t median;
	
	ADC_t *sensor_adc;
	uint8_t sensor_MUX_gc;

	// figure out if a new measurement is necessary
	current_time = get_milliseconds();
	
	//if((!APS_ADC_data_is_fresh(actuator_num))
	//if((APS_init_complete)&&(!APS_ADC_data_is_fresh(actuator_num)))
	if((APS_ADC_DEBUG_LEVEL > 0)&&(!APS_ADC_data_is_fresh(actuator_num)))
		printf("$%u", actuator_num);	// debug, measurement refresh rate too slow
	
	last_APS_ADC_meas_time_ms[act_ind] = current_time;	// global assignment

	switch(actuator_num)
	{
		case 1:	sensor_adc = &APS1_ADC;	sensor_MUX_gc = APS1_MUXPOS_gc;	break;
		case 2:	sensor_adc = &APS2_ADC;	sensor_MUX_gc = APS2_MUXPOS_gc;	break;
		case 3:	sensor_adc = &APS3_ADC;	sensor_MUX_gc = APS3_MUXPOS_gc;	break;
		case 4:	sensor_adc = &APS4_ADC;	sensor_MUX_gc = APS4_MUXPOS_gc;	break;
		case 5:	sensor_adc = &APS5_ADC;	sensor_MUX_gc = APS5_MUXPOS_gc;	break;
		case 6:	sensor_adc = &APS6_ADC;	sensor_MUX_gc = APS6_MUXPOS_gc;	break;
		default:	return 0;
	}
	
	// (re)connect the MUX:
	(*sensor_adc).CH0.MUXCTRL &= ~0b01111000;	// clear out the old MUXPOS_gc
	(*sensor_adc).CH0.MUXCTRL |= sensor_MUX_gc;

	for (uint8_t i = 0; i < 3; i++)
	{
		(*sensor_adc).CTRLA |= ADC_CH0START_bm;
		while ((*sensor_adc).CH0.INTFLAGS==0){};		// wait for 'complete flag' to be set
		(*sensor_adc).CH0.INTFLAGS = 1;				// clear the complete flag 
		meas[i] = (*sensor_adc).CH0.RES;
	}		

	median = find_median_of_3(meas);	// note, could also do average if desired

	if(APS_ADC_DEBUG_LEVEL > 0)	printf(">");	// DEBUG
	
	// save this measurement for future use
	last_APS_ADC_measurement[act_ind] = median;		// global assignment
	
	return median;
}


uint8_t guess_pressure_sensor_missing(uint8_t act_num)
{
	request_overtime_permission();
	release_acutator_pressure_meas_lock();
	
	if(APS_SELFCHECK_DEBUG_PRINT > 0)
	{
		printf("\r\nAct%u APS Self Diag\r\n", act_num);
		_delay_ms(5);
	}
	
	// theory of operation:
	// If a sensor is actually connected to the ADC input, the sensor circuitry will hold the voltage to a well defined level.
	// If a sensor is not connected to the ADC input, the input voltage will be floating.
	// A floating voltage will usually appear as a value of 75 (with current ADC settings).
	// With rapid repeated measurements though, the capacitor of the ADC input will begin to accumulate charge, and this value will drift up to around 82.
	// A connected sensor will not exhibit this drift (and will also have a lower initial voltage value (~11).
	// Therefore, look for this drift to detect if a sensor is actually attached to the ADC input.

	// typical findings from floating sequence:
	// actuator (ADC) number (these for the first-built board), 20 measurements, no delay except for print
	// 1| 4 0 0 0 0 1 -1 1 -1 0 1 -1 0 1 -1 0 1 -1 0 0
	// 2| 19 5 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
	// 3| 9 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 -1 1 -1 1
	// 4| 14 4 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
	// 5| 8 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
	// 6| 6 -1 2 0 -1 5 -2 -5 3 3 -2 -4 3 2 -1 -4 3 2 -1 -4

	uint8_t drift_count;
	uint8_t evidence_for_missing = 0;
	uint8_t pass_fail_flags = 0;
	uint8_t test_pass; // boolean, reset for each test

	uint8_t meas_values[DRIFT_CHECK_NUM_ITERATIONS];

	// COLLECT THE TEST DATA  ----------------------------------------------------------------------
	
	drift_count = 0;

	// take the initial measurement
	meas_values[0] = get_actuator_pressure_ADC(act_num);

	for(uint8_t i = 1; i < DRIFT_CHECK_NUM_ITERATIONS; i++)
	{
		meas_values[i] = get_actuator_pressure_ADC(act_num);

		// look for an increase of 2+ from previous reading
		if(meas_values[i] > meas_values[i-1]+1)
			drift_count++;

		// look for a drop of 2+ from previous reading
		if(meas_values[i]+1 < meas_values[i-1])	// negative change indicates oscillations
			drift_count++;
	}

	// print the sequence of DELTAs, a stable reading will be all 0s
	if(APS_SELFCHECK_DEBUG_PRINT >= 3)
	{
		printf("TEST DATA: ");
		for(uint8_t i = 0; i < DRIFT_CHECK_NUM_ITERATIONS; i++)	
			printf("%i ", meas_values[i]);

		printf("\r\n");
		_delay_ms(5);
	}


	// TEST1 - INITIAL VALUE ----------------------------------------------------------------------
	
	test_pass = 0;
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
	{
		printf("TEST1 - initial value\r\n");
		_delay_ms(5);
	}
	
	if(APS_SELFCHECK_DEBUG_PRINT == 3)
		printf("%u outside range [60, 100], ", meas_values[0]);
		
	if((meas_values[0] > 60) && (meas_values[0] < 100))
	{
		evidence_for_missing++;
		pass_fail_flags += (1<<0);
	}
	else	test_pass = 1;
		
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
		if(test_pass)	printf("PASS\r\n");
		else			printf("FAIL\r\n");
			
	// TEST2 - INITIAL DRIFT CHECK ----------------------------------------------------------------------
	
	test_pass = 0;
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
	{
		printf("TEST2 - initial drift\r\n");
		_delay_ms(5);	
	}
	
	if(APS_SELFCHECK_DEBUG_PRINT == 3)
		printf("%i < 2, ", meas_values[1] - meas_values[0]);

	// if the 2nd measurement is 2+ higher than the first, this almost assuredly indicates a disconnected sensor
	if(meas_values[1] - meas_values[0] >= 2)
	{
		pass_fail_flags += (1<<1);
		evidence_for_missing++;
	}
	else	test_pass = 1;
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
		if(test_pass)	printf("PASS\r\n");
		else			printf("FAIL\r\n");
	
	// TEST3 - OVERALL DRIFT CHECK ----------------------------------------------------------------------

	test_pass = 0;

	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
	{
		printf("TEST3 - overall drift\r\n");
		_delay_ms(5);
	}

	if(APS_SELFCHECK_DEBUG_PRINT == 3)
		printf("%i < 5, ", meas_values[DRIFT_CHECK_NUM_ITERATIONS-1] - meas_values[0]);

	// compare the first measurement to the last measurement
	if(meas_values[DRIFT_CHECK_NUM_ITERATIONS-1] - meas_values[0] >= 5)	// Large positive drift
	{
		evidence_for_missing++;
		pass_fail_flags += (1<<2);
	}
	else	test_pass = 1;
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
		if(test_pass)	printf("PASS\r\n");
		else			printf("FAIL\r\n");
	
	// TEST4 - FINAL VALUE CHECK ----------------------------------------------------------------------

	test_pass = 0;

	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
	{
		printf("TEST4 - final value\r\n");
		_delay_ms(5);
	}

	if(APS_SELFCHECK_DEBUG_PRINT == 3)
		printf("%u outside range [83, 88], ", meas_values[DRIFT_CHECK_NUM_ITERATIONS-1]);

	if((meas_values[DRIFT_CHECK_NUM_ITERATIONS-1] > 83) && (meas_values[DRIFT_CHECK_NUM_ITERATIONS-1] < 88))
	{
		evidence_for_missing++;
		pass_fail_flags += (1<<3);
	}
	else	test_pass = 1;
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
		if(test_pass)	printf("PASS\r\n");
		else			printf("FAIL\r\n");
		
	// TEST5 - OSCIALLTIONS ----------------------------------------------------------------------
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
	{
		printf("TEST5 - oscillations\r\n");
		_delay_ms(5);
	}
	
	// print the sequence of DELTAs, a stable reading will be all 0s
	if(APS_SELFCHECK_DEBUG_PRINT >= 3)
	{
		for(uint8_t i = 1; i < DRIFT_CHECK_NUM_ITERATIONS-1; i++)
			printf("%i ", meas_values[i] - meas_values[i-1]);

		printf("\r\n");
	}

	if(APS_SELFCHECK_DEBUG_PRINT == 3)
		printf("%i <= %i, ", drift_count, DRIFT_CHECK_NUM_ITERATIONS/4);
	
	if(drift_count > DRIFT_CHECK_NUM_ITERATIONS/4)
	{
		evidence_for_missing++;
		pass_fail_flags += (1<<4);
	}
	else	test_pass = 1;
	
	if(APS_SELFCHECK_DEBUG_PRINT >= 2)
		if(test_pass)	printf("PASS\r\n");
		else			printf("FAIL\r\n");
		
	if(APS_SELFCHECK_DEBUG_PRINT == 1)
	{
		for(uint8_t test_num = 0; test_num < 5; test_num++)
		{
			if((pass_fail_flags>>test_num & 1) == 1)	printf("F");
			else										printf("P");
		}
		printf("\r\n");
	}	

	if(evidence_for_missing == 0)
		if(APS_SELFCHECK_DEBUG_PRINT >= 2) 
			printf("Sensor OK\r\n");
	_delay_ms(5);
		
	set_acutator_pressure_meas_lock();
	return evidence_for_missing;
}


uint8_t is_ADC_pressure_calibrated(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;

	return (1 << (act_ind)) & ADC_pressure_calib_flags;
}

void set_ADC_pressure_calibrated(uint8_t act_num, uint8_t calibrated)
{
	uint8_t act_ind = act_num - 1;

	if(calibrated)
		ADC_pressure_calib_flags |= (1 << act_ind);
	else
		ADC_pressure_calib_flags &=~ (1 << act_ind);
}

void save_new_ADCtoPSI_calibration(uint8_t act_num, float slope, float yint)
{
	printf("save|act%u\r\n", act_num);
	_delay_ms(5);
		
	uint8_t act_ind = act_num - 1;
	
	calibration_ADCtoPSI_slope[act_ind] = slope;

	calibration_ADCtoPSI_yint[act_ind] = yint;

	set_ADC_pressure_calibrated(act_num, 1);
}

float get_actuator_pressure_PSI(uint8_t act_num)
{
	float pressure = 0;
	uint8_t ADC_pressure;

	if(is_ADC_pressure_calibrated(act_num))
	{
		if(!APS_ADC_data_is_fresh(act_num))
			printf("STALE#%u", act_num);	
		
		ADC_pressure = get_actuator_pressure_last_ADC(act_num);
		
		if(ADC_pressure <= 15)
			pressure = 0; // effectively true, though this is very non-linear (hack?) may cause trouble
		else
			pressure = calibration_ADCtoPSI_yint[act_num-1] + calibration_ADCtoPSI_slope[act_num-1]*ADC_pressure;	
	}
	else
		print_actuator_pressure_equation(act_num); // this prints the error message for uncalibrated
	
	// FIX FOR VOLTAGE DROP DUE TO PUMP AND VALVES:
	pressure = pressure_drag_drop_correction(pressure, act_num);

	//printf("P%f\r\n", pressure);	// DEBUG - super fast printing!

	return pressure;
}

uint8_t APS_ADC_data_is_fresh(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;
	uint16_t current_time = get_milliseconds();
	uint16_t t_diff = compute_tdiff_ms(last_APS_ADC_meas_time_ms[act_ind], current_time);
	
	if(t_diff > APS_DATA_EXPIRATION_TIME_ms)
	{
		return 0;
	}
	
	return 1;
}

uint8_t get_actuator_pressure_last_ADC(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;

	if(!APS_ADC_data_is_fresh(act_num))
	{
		printf("WARNING: stale data\r\n");
		_delay_ms(5000);
	}


	return last_APS_ADC_measurement[act_ind];
}

void print_actuator_pressure_equation(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;
	
	if(is_ADC_pressure_calibrated(act_num))
		print_precise_line_equation(calibration_ADCtoPSI_slope[act_ind], calibration_ADCtoPSI_yint[act_ind]);

	else
	{
		printf("Actuator %u uncalibrated", act_num);
		_delay_ms(3);
		printf(" NO LINE\r\n");
		_delay_ms(3);	
	}
}
