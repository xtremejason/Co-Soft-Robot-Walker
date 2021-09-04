//SRpressure_control.h

#ifndef __SR_PRESSURE_CONT_H__
#define __SR_PRESSURE_CONT_H__


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h> // used for stdout(?), printf

#include "../SoftRobotSource/SRanalog_pressure.h"
#include "../SoftRobotSource/SRdigital_pressure.h"
#include "../SoftRobotSource/SRvalves.h"
#include "../SoftRobotSource/SRpump.h"
#include "../SoftRobotSource/SRmisc_functions.h" // mathematica print, OLS solver


#ifdef __cplusplus
extern "C"
{
#endif


// CONVENTIONS: act_num = [1, ... 6]


/* USER ACCESSIBLE FUNCTIONS */

void actuator_set_and_forget(uint8_t actuator_num, float target_pressure);

void set_pump_speed_manual(uint8_t speed);

uint8_t get_pump_speed_manual(void);

uint8_t get_pump_speed_automatic(void);

float get_maximum_actuator_set_pressure(void);


/* INTERNAL FUNCTIONS */

// called in the interrupt ONLY (TCE0_OVF_vect @ 125 Hz)
uint8_t service_actuators(void);

void stop_actuator_service(uint8_t actuator_num);

void start_actuator_service(uint8_t actuator_num, float goal_pressure);

void set_actuator_inflating(uint8_t act_num, uint8_t set_on);

uint8_t actuator_is_inflating(uint8_t act_num);

void set_actuator_deflating(uint8_t act_num, uint8_t set_on);

uint8_t actuator_is_deflating(uint8_t act_num);

void set_target_pressure(uint8_t act_num, float target_pressure);

float get_target_pressure(uint8_t act_num);

void set_pump_speed_automatic(uint8_t speed);

void set_pump_mode(uint8_t new_mode);

void toggle_pump_mode(void);


/* CALIBRATION */

// running the calibration routine saves a calibration line and enables the actuator
void calibrate_pressure_sensor(uint8_t actuator_num);	// BLOCKING!
void calibrate_pressure_sensor_debug(uint8_t actuator_num); // a variant, adds in pump or no-valves

// run the calibration routine on the entire robot, takes awhile
void build_entire_new_calibration(void);					// BLOCKING!

void take_pressure_pair(uint8_t *ADC, float *DP, uint8_t sensor_num);


/* INITIALIZATION */

void init_pressure_control(void);	// turns on TCE0 for interrupt control


/* INTERRUPTS */

// ISR(TCE0_OVF_vect)	// 125 Hz

#ifdef __cplusplus
}
#endif

#endif