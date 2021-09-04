//SRpressure_control.c

#include "SRpressure_control.h"

#include <util/delay.h>

//#include "SRreservoir_control.h"
#include "SRmotion_control.h"
#include "../SoftRobotSource/SRstatus_LED.h"	// for toggle_LED(), in interrupt


// calibration timings
#define ZERO_SEARCH_INCREMENT_ms			   5
#define CALIBRATION_VALVE_OPEN_INCREMENT_ms	  40
#define FILL_INCREMENT_ms					  10
#define VENT_INCREMENT_ms					  10
#define HOLD_INCREMENT_ms					  50
#define PRESSURIZING_INCREMENT_ms			 500
#define STABILIZE_INCREMENT_ms				3000 // 1000 seems no different than 3000 (NDF - Nov 29, 2015)
#define EVACUATE_TIME_ms					5000 // (minimum) time to completely empty air pressure to zero (est.)

#define NUM_PRESSURE_C_MEAS					  14 // number of data points collected for calibration
#define CALIBRATION_MAX_PRESSURE_psi		 8.0 // calibration routine pressurizes up to this, to compute line equation
//#define ACTUATOR_MAX_PRESSURE_psi			 8.5 // users cannot set an actuator pressure higher than this
#define ACTUATOR_MAX_PRESSURE_psi			 9.0 // users cannot set an actuator pressure higher than this
#define ACTUATOR_PRESSURE_TOLERANCE_psi		0.10 // UNUSED - previously we were servoing the pressures, may re-implement

#define PUMP_MODE_MANUAL			0
#define PUMP_MODE_AUTOMATIC			1

#define PRESSURE_CONTROL_DEBUG  	0	// no printing
//#define PRESSURE_CONTROL_DEBUG  	1	// print START/STOP actions & pump mode changes
//#define PRESSURE_CONTROL_DEBUG  	2	// print arrows -->, every timestep

// flags:
uint8_t actuator_inflating_flags = 0;	// bit set if actuator is currently inflating (fill valve open, pump on, p_meas < p_set)  
uint8_t actuator_deflating_flags = 0;	// bit set if actuator is currently deflating (vent valve open, p_meas > p_set)

float set_pressure[6];		// keeps track of what the pressure *should* be in each actuator, in PSI

uint8_t manual_pump_speed = PUMP_SPEED_OFF;
uint8_t automatic_pump_speed = PUMP_SPEED_DEFAULT;
uint8_t pump_mode = PUMP_MODE_AUTOMATIC;

uint8_t TCE0_OVF_counter = 0;	// for down-sampling interrupt rate

// [TODO] KNOWN BUG:
// When actuator is inflating to a higher pressure than it is at, the pump will turn on (normal)
// If an air hose fails during inflation (abnormal) the pressure in the actuator will drop to zero
// If a command is then sent to set the pressure to zero, in an attempt to shut off the pump
// The system will recognize that it is already at the set pressure, and thus takes no action
// This results in the pump remaining in the on-state, with the set pressure equal to zero.
// Consider checking for measured = set = target = 0, and manually turn pump off here
// However, it is unknown if the other actuator settings would turn the pump bcak on again?


/* USER ACCESSIBLE FUNCTIONS */

void actuator_set_and_forget(uint8_t actuator_num, float target_pressure)
{
	// keep pressures to an acceptable range
	if(target_pressure > ACTUATOR_MAX_PRESSURE_psi)
	target_pressure = ACTUATOR_MAX_PRESSURE_psi;
	if(target_pressure < 0)
	target_pressure = 0;
	
	if(is_ADC_pressure_calibrated(actuator_num))
	{
		start_actuator_service(actuator_num, target_pressure);
		// (TODO) check that the reservoir has sufficient pressure to fill the new actuator
	}
	
	else
	printf("-cant service NC-\r\n");
}

void set_pump_speed_manual(uint8_t speed)
{
	if(pump_mode == PUMP_MODE_MANUAL)
	{
		manual_pump_speed = speed;	// PRIVATE VARIABLE ASSIGNMENT

		set_pump_power(manual_pump_speed);
	}
}

uint8_t get_pump_speed_manual(void)
{
	return manual_pump_speed;
}

uint8_t get_pump_speed_automatic(void)
{
	return automatic_pump_speed;
}

float get_maximum_actuator_set_pressure(void)
{
	float max = 0;

	for(uint8_t i = 0; i < 6; i++)
	if(set_pressure[i] > max)
	max = set_pressure[i];

	return max;
}


/* INTERNAL FUNCTIONS */

uint8_t service_actuators(void)
{
	float pressure;
	float target_pressure;

	uint8_t pump_is_needed = 0;			// boolean
	uint8_t something_is_serviced = 0;	// boolean
	
	release_acutator_pressure_meas_lock();
	
	// always measure the pressure:
	for(uint8_t act_num = 1; act_num < 7; act_num++)
	{
		get_actuator_pressure_ADC(act_num);
		// the value found here will be retrieved again at the next call of
		// get_actuator_pressure_PSI(), in the code just below!
	}
	
	set_acutator_pressure_meas_lock();
	
	for(uint8_t act_num = 1; act_num < 7; act_num++)
	{
		// to keep timing consistent, always measure the pressure, even if not needed
		if(is_ADC_pressure_calibrated(act_num))
		{
			pressure = get_actuator_pressure_PSI(act_num);
			target_pressure = get_target_pressure(act_num);

			if(actuator_is_inflating(act_num))
			{
				something_is_serviced = 1;
				if(PRESSURE_CONTROL_DEBUG >= 2)	printf("%u -->\r\n", act_num);	// DEBUG!

				// check if we can stop inflating
				if(pressure > target_pressure)
				stop_actuator_service(act_num);
				else
				pump_is_needed = 1;	// we are not at pressure, use the pump for this one
			}
			
			if(actuator_is_deflating(act_num))
			//else if(actuator_is_deflating(act_num))
			{
				something_is_serviced = 1;
				if(PRESSURE_CONTROL_DEBUG >= 2)	printf("%u <--\r\n", act_num);	// DEBUG!
				
				// check if we can stop evacuating air
				if(pressure <= target_pressure)
				{
					stop_actuator_service(act_num);
				}
			}
		}
	}
	
	if(pump_is_needed)	set_pump_speed_automatic(PUMP_SPEED_HI);
	else				set_pump_speed_automatic(PUMP_SPEED_OFF);

	return something_is_serviced;
}

void stop_actuator_service(uint8_t actuator_num)
{
	float pressure;
	
	if(PRESSURE_CONTROL_DEBUG >= 1)
	printf("STOP %u", actuator_num);
	
	pressure = get_actuator_pressure_PSI(actuator_num);
	
	if(PRESSURE_CONTROL_DEBUG >= 1)
	printf(" hold @ [%.2f]\r\n", (double)pressure);
	
	valve_hold(actuator_num);	// always hold air when service is ended
	
	_delay_ms(5);	// DEBUG
	
	set_actuator_inflating(actuator_num, 0);
	set_actuator_deflating(actuator_num, 0);

	// for motion control
	if(motion_in_progress())
	clear_actuator_motion_checkpoint(actuator_num);
}

void start_actuator_service(uint8_t actuator_num, float goal_pressure)
{
	float pressure;
	
	if(PRESSURE_CONTROL_DEBUG >= 1)
	printf("START %u", actuator_num);
	
	pressure = get_actuator_pressure_PSI(actuator_num);
	
	// save the desired pressure
	//set_pressure[act_ind] = goal_pressure;
	set_target_pressure(actuator_num, goal_pressure);
	
	if(pressure < goal_pressure)
	{
		if(PRESSURE_CONTROL_DEBUG >= 1)
		printf(" @ [%.2f] +> [%.2f]\r\n", (double)pressure, (double)goal_pressure);
		
		set_actuator_inflating(actuator_num, 1);
		set_actuator_deflating(actuator_num, 0);
		valve_fill(actuator_num);
	}
	else if(pressure > goal_pressure)
	{
		if(PRESSURE_CONTROL_DEBUG >= 1)
		printf(" @ [%.2f] -> [%.2f]\r\n", (double)pressure, (double)goal_pressure);
		
		set_actuator_deflating(actuator_num, 1);
		set_actuator_inflating(actuator_num, 0);
		valve_vent(actuator_num);
	}
	else	// already at desired pressure
	{
		if(PRESSURE_CONTROL_DEBUG >= 1)
		printf(" @ [%.2f] == [%.2f]\r\n", (double)pressure, (double)goal_pressure);
		
		valve_hold(actuator_num);
		
		// for motion control
		if(motion_in_progress())
		clear_actuator_motion_checkpoint(actuator_num);
	}
	
	_delay_ms(5);	// DEBUG
}

void set_actuator_inflating(uint8_t act_num, uint8_t set_on)
{
	uint8_t act_ind = act_num - 1;
	
	if(set_on)
	actuator_inflating_flags |= (1 << act_ind);
	else
	actuator_inflating_flags &=~ (1 << act_ind);
}

uint8_t actuator_is_inflating(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;
	
	if(actuator_inflating_flags & (1 << act_ind))
	return 1;
	
	return 0;
}

void set_actuator_deflating(uint8_t act_num, uint8_t set_on)
{
	uint8_t act_ind = act_num - 1;
	
	if(set_on)
	actuator_deflating_flags |= (1 << act_ind);
	else
	actuator_deflating_flags &=~ (1 << act_ind);
}

uint8_t actuator_is_deflating(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;
	
	if(actuator_deflating_flags & (1 << act_ind))
	return 1;
	
	return 0;
}

void set_target_pressure(uint8_t act_num, float target_pressure)
{
	uint8_t act_ind = act_num - 1;
	
	set_pressure[act_ind] = target_pressure;
}

float get_target_pressure(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;

	return set_pressure[act_ind];
}

void set_pump_speed_automatic(uint8_t speed) // PRIVATE FUNCTION
{
	if(pump_mode == PUMP_MODE_AUTOMATIC)
	{
		automatic_pump_speed = speed;	// PRIVATE VARIABLE ASSIGNMENT

		set_pump_power(automatic_pump_speed);
	}
}

void set_pump_mode(uint8_t new_mode)
{
	pump_mode = new_mode;

	if(PRESSURE_CONTROL_DEBUG >= 1)
	{
		printf("PUMP->");
		if(pump_mode == PUMP_MODE_AUTOMATIC)
		printf("AUTOMATIC\r\n");
		else if(pump_mode == PUMP_MODE_MANUAL)
		printf("MANUAL\r\n");
		else
		printf("ERROR\r\n");
	}
}

void toggle_pump_mode(void)
{
	if(pump_mode == PUMP_MODE_AUTOMATIC)
		set_pump_mode(PUMP_MODE_MANUAL);
	else
		set_pump_mode(PUMP_MODE_AUTOMATIC);
}


/* CALIBRATION */

void calibrate_pressure_sensor(uint8_t actuator_num)
{
	uint8_t analog_pressure_x_points[NUM_PRESSURE_C_MEAS];
	float digital_pressure_y_points[NUM_PRESSURE_C_MEAS];

	float temp_DP;		// digital pressure
	uint8_t temp_AP;	// analog pressure

	float slope, yint;
	//uint8_t stretch_sensor;

	// STEP 0)
	//	take over manual control of the pump

	//reservoir_valve_close();

	set_pump_speed_manual(PUMP_SPEED_OFF);

	_delay_ms(5);
	printf("calibrate %u->\r\n", actuator_num);
	_delay_ms(10);
	
	//print_actuator_pressure_equation(actuator_num);	// DEBUG: see current line before overwriting

	// STEP 1)
	//  open fill valve, close vent valve
	//	turn the pump on with a modest speed (manual mode)
	//	fill actuator to max pressure that pump can supply (or less)
	//	this will tend to be ~8.5 PSI (make sure that the actuator doesn't burst!)

	printf("STEP1 - inflate\r\n");
	_delay_ms(10);

	valve_fill(actuator_num);
	_delay_ms(PRESSURIZING_INCREMENT_ms);
	
	set_pump_speed_manual(PUMP_SPEED_HI);

	do{
		take_pressure_pair(&temp_AP, &temp_DP, actuator_num);
		printf("#P %f\r\n", (double)temp_DP);
		_delay_ms(PRESSURIZING_INCREMENT_ms);
	
	}while(temp_DP < CALIBRATION_MAX_PRESSURE_psi);

	// STEP 2)
	//  keeping fill valve open, shut off the pump
	//  wait until pressure stabilizes (~ 1 sec ???)
	
	printf("STEP2 - stabilize\r\n");
	
	set_pump_speed_manual(PUMP_SPEED_OFF);
	_delay_ms(STABILIZE_INCREMENT_ms);

	// STEP 3)
	//  open vent valve for few milliseconds (you choose)
	//  close vent valve, wait until pressure stabilizes (~ 1 sec ???)
	//  measure the pressure pair
	//  repeat step 3 for 10 or so measurements

	printf("STEP3 - take data\r\n");
	
	//printf("#   P-adc P-flt  stretch\r\n");
	printf("#   P-adc P-flt\r\n");

	for(uint8_t i = 0; i < NUM_PRESSURE_C_MEAS; i++)
	{
		take_pressure_pair(&analog_pressure_x_points[i], &digital_pressure_y_points[i], actuator_num);
		
		printf("%02u  %03u  %1.4f\r\n", i, analog_pressure_x_points[i], (double)digital_pressure_y_points[i]);

		if(i < NUM_PRESSURE_C_MEAS-1)	// if there is another measurement to take
		{	
			// let a little bit of air out of the actuator/pump/reservoir tube-network
			valve_thru_open(actuator_num);
			_delay_ms(CALIBRATION_VALVE_OPEN_INCREMENT_ms);				
			valve_fill(actuator_num);
			_delay_ms(STABILIZE_INCREMENT_ms);
		}	
	}
	
	// STEP 4)
	//	do math to complete calibration routine

	printf("STEP4 - OLS solver\r\n");
	
	OLS_solver(analog_pressure_x_points, digital_pressure_y_points, NUM_PRESSURE_C_MEAS, &slope, &yint);

	// check for a bad calibration
	if((slope < 0.02)||
		(slope != slope))	// this comparison true if slope is 'nan' (divide by zero error)
	{	
		// bad calibration
		// TODO: find a more robust check for failure (use StdDev of points or something)
		printf("bad calibration\r\n");
	}

	else
	{
		save_new_ADCtoPSI_calibration(actuator_num, slope, yint);

		printf("NEW SAVE\r\n");
		print_actuator_pressure_equation(actuator_num);
	}

	// print formatted for Mathematica
	print_mathematica_formatted_2column(analog_pressure_x_points, digital_pressure_y_points, NUM_PRESSURE_C_MEAS);
	
	// STEP 5)
	//	close fill valve, open vent valve
	//  completely empty actuator
	//  close vent valve
	
	printf("STEP5 - evacuate\r\n");
	
	valve_vent(actuator_num);
	_delay_ms(EVACUATE_TIME_ms);	// 3 seconds seems reasonable
	valve_hold(actuator_num);
	
	// STEP 6)
	//	reenable automatic control of the pump
}

void calibrate_pressure_sensor_debug(uint8_t actuator_num)
{
	// took out the references to strain sensor, added functions to measure the effect of
	// (1) the pump being on while taking an ADC measurement (needs a person to hold the hose closed)
	// (2) the valves being off while taking an ADC measurement (previously, the fill valve was left open)
	// part (1) is currently commented, or rather part (2) was put in, in place of part (1)
	// It is noted that the valves actually draw a bit of current, and so their effect is non-negligible
	
	uint8_t analog_pressure_x_points[NUM_PRESSURE_C_MEAS];
	uint8_t p_analog_pressure_x_points[NUM_PRESSURE_C_MEAS];
	float digital_pressure_y_points[NUM_PRESSURE_C_MEAS];
	float p_scrap;

	float temp_DP;		// digital pressure
	uint8_t temp_AP;	// analog pressure

	float slope, yint;
	float p_slope, p_yint;
	
	
	// STEP 0)
	//	take over manual control of the pump

	//reservoir_valve_close();

	set_pump_speed_manual(PUMP_SPEED_OFF);

	_delay_ms(5);
	printf("calibrate %u->\r\n", actuator_num);
	_delay_ms(10);
	
	//print_actuator_pressure_equation(actuator_num);	// DEBUG: see current line before overwriting

	// STEP 1)
	//  open fill valve, close vent valve
	//	turn the pump on with a modest speed (manual mode)
	//	fill actuator to max pressure that pump can supply (or less)
	//	this will tend to be ~8.5 PSI (make sure that the actuator doesn't burst!)

	printf("STEP1 - inflate\r\n");
	_delay_ms(10);

	valve_fill(actuator_num);
	_delay_ms(PRESSURIZING_INCREMENT_ms);
	
	set_pump_speed_manual(PUMP_SPEED_HI);

	do{
		take_pressure_pair(&temp_AP, &temp_DP, actuator_num);
		printf("#P %f\r\n", (double)temp_DP);
		_delay_ms(PRESSURIZING_INCREMENT_ms);
		
	}while(temp_DP < CALIBRATION_MAX_PRESSURE_psi);

	// STEP 2)
	//  keeping fill valve open, shut off the pump
	//  wait until pressure stabilizes (~ 1 sec ???)
	
	printf("STEP2 - stabilize\r\n");
	
	set_pump_speed_manual(PUMP_SPEED_OFF);
	_delay_ms(STABILIZE_INCREMENT_ms);

	printf("disconnect pump..");
	for(uint8_t t = 10; t > 0; t--)
	{
		_delay_ms(500);
		printf("%u..", t);
	}
	printf("\r\n");

	// STEP 3)
	//  open vent valve for few milliseconds (you choose)
	//  close vent valve, wait until pressure stabilizes (~ 1 sec ???)
	//  measure the pressure pair
	//  repeat step 3 for 10 or so measurements

	printf("STEP3 - take data\r\n");
	
	printf("#   P-adc P-flt  P-adc(w/pump)\r\n");
	for(uint8_t i = 0; i < NUM_PRESSURE_C_MEAS; i++)
	{
		take_pressure_pair(&analog_pressure_x_points[i], &digital_pressure_y_points[i], actuator_num);
		
		_delay_ms(PRESSURIZING_INCREMENT_ms);
		//set_manual_pump_speed(PUMP_SPEED_HI);
		valve_hold(actuator_num);
		_delay_ms(PRESSURIZING_INCREMENT_ms);
	
		take_pressure_pair(&p_analog_pressure_x_points[i], &p_scrap, actuator_num);
		
		printf("%02u  %03u  %1.4f  %u\r\n", i, analog_pressure_x_points[i], (double)digital_pressure_y_points[i],  p_analog_pressure_x_points[i]);

		_delay_ms(PRESSURIZING_INCREMENT_ms);
		//set_manual_pump_speed(PUMP_SPEED_OFF);
		valve_fill(actuator_num);	//debugging
		_delay_ms(PRESSURIZING_INCREMENT_ms);

		if(i < NUM_PRESSURE_C_MEAS-1)	// if there is another measurement to take
		{	
			// let a little bit of air out of the actuator/pump/reservoir tube-network
			valve_thru_open(actuator_num);
			_delay_ms(CALIBRATION_VALVE_OPEN_INCREMENT_ms);
			valve_fill(actuator_num);
			_delay_ms(STABILIZE_INCREMENT_ms);
		}
	}
	
	// STEP 4)
	//	do math to complete calibration routine

	printf("STEP4 - OLS solver\r\n");
	
	OLS_solver(analog_pressure_x_points, digital_pressure_y_points, NUM_PRESSURE_C_MEAS, &slope, &yint);
	OLS_solver(p_analog_pressure_x_points, digital_pressure_y_points, NUM_PRESSURE_C_MEAS, &p_slope, &p_yint);

	printf("NO SAVE!\r\n");

	print_precise_line_equation(slope, yint);
	print_mathematica_formatted_2column(analog_pressure_x_points, digital_pressure_y_points, NUM_PRESSURE_C_MEAS);

	print_precise_line_equation(p_slope, p_yint);
	print_mathematica_formatted_2column(p_analog_pressure_x_points, digital_pressure_y_points, NUM_PRESSURE_C_MEAS);

	
	// STEP 5)
	//	close fill valve, open vent valve
	//  completely empty actuator
	//  close vent valve
	
	printf("STEP5 - evacuate\r\n");
	
	valve_vent(actuator_num);
	_delay_ms(EVACUATE_TIME_ms);	// 3 seconds seems reasonable
	valve_hold(actuator_num);
	
	printf("reconnect pump!");
	
	// STEP 6)
	//	reenable automatic control of the pump
}

void build_entire_new_calibration(void)
{
	printf("Build new setup\r\n");
	
	for(uint8_t actuator_num = 1; actuator_num <= 6; actuator_num++)
	{
		if(!guess_pressure_sensor_missing(actuator_num))
		calibrate_pressure_sensor(actuator_num);
	}
	
	return;
}

// will only work for this if air hoses are hooked up correctly
void take_pressure_pair(uint8_t *ADC, float *DP, uint8_t sensor_num)
{
	// should be monitoring pressures continuously in interrupt loop
	*ADC = get_actuator_pressure_last_ADC(sensor_num);
	
	*DP = get_digital_pressure();
}


/* INITIALIZATION */

void init_pressure_control(void)
{
	// Set TCE0 for low-level overflow interrupt at ~ 125 Hz
	TCE0.CTRLA = TC_CLKSEL_DIV1024_gc;
	TCE0.CTRLB = 0;
	TCE0.CTRLC = 0;
	TCE0.CTRLD = 0;
	TCE0.CTRLE = 0;
	TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;		// this is an overflow interrupt (ref: doc8331.pdf, pg 181)
	TCE0.INTCTRLB = 0;						// this is for clock compare interrupts (there are 4 avail)
	TCE0.CNT = 0;
	TCE0.PER = 250;	// VERIFIED: this is 125 +/- 1 Hz (4/1/16)

	//set_max_reservoir_pressure(6);
	//set_min_reservoir_pressure(4);
	
	if(pump_mode == PUMP_MODE_AUTOMATIC)
		printf(" Automatic Pump\r\n");
	else // pump_mode = PUMP_MODE_MANUAL
		printf(" Manual Pump\r\n");
}


/* INTERRUPTS */

ISR(TCE0_OVF_vect)	// 125 Hz
{
	// this ISR should still execute despite a crash-freeze of the main loop
		
	service_actuators();

	TCE0_OVF_counter++;
	TCE0_OVF_counter%=125;
	if(TCE0_OVF_counter == 0)
	{
		toggle_LED(1);
	}
}

