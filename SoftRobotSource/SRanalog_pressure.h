// SRanalog_pressure.h

#ifndef __SRANALOG_PRESSURE_H__
#define __SRANALOG_PRESSURE_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif

//#define APS_SELFCHECK_DEBUG_PRINT 0		// silent
#define APS_SELFCHECK_DEBUG_PRINT 1		// ONLY print pass/fail symbols
//#define APS_SELFCHECK_DEBUG_PRINT 2		// print commentary
//#define APS_SELFCHECK_DEBUG_PRINT 3		// print values


// CONVENTIONS:
// actuator_num = {1, ... 6} AKA act_num
// act_ind = {0, ... 5} AKA actuator_index AKA act_index
// act_ind = act_num - 1 **ALWAYS**
// For the sake of accuracy with function calls ALL functions take as input the actuator number!
// If an actuator index is needed (within a function) it is computed on the spot in the function.
// Every attempt was made to remove any shortcut uses of act_ind, Example:
// ** BAD **	val = array_val[act_num-1];
// ** GOOD **	uint8_t act_ind = act_num-1;	val = array_val[act_ind];  


/* INITIALIZATION */

void APS_init(void);



/* USER ACCESSIBLE FUNCTIONS */

uint8_t get_actuator_pressure_ADC(uint8_t actuator_num);
// note: returns ADC_LOWEST_READING for any v-meas that is below the "activation voltage"

float get_actuator_pressure_PSI(uint8_t act_num);

void save_new_ADCtoPSI_calibration(uint8_t act_num, float slope, float yint);

void print_actuator_pressure_equation(uint8_t act_num);

// these should ONLY be called within the service_actuators() function!
// this is to prevent 'user code' from requesting a new measurement outside of interrupt driven the control loop
void set_acutator_pressure_meas_lock(void);
void release_acutator_pressure_meas_lock(void);


/* INTERNAL FUNCTIONS */

uint8_t APS_ADC_data_is_fresh(uint8_t act_num);

uint8_t get_actuator_pressure_last_ADC(uint8_t act_num);

uint8_t is_ADC_pressure_calibrated(uint8_t act_num);

// boolean toggle, set to approve the calibration equation that is saved
// calibrated: 1 = set as calibrated, 0 = set as uncalibrated
void set_ADC_pressure_calibrated(uint8_t act_num, uint8_t calibrated);


// check if there is a pressure sensor attached to the ADC input
// performs a series of tests (currently 5) to check the presence of a load on the sensor input
uint8_t guess_pressure_sensor_missing(uint8_t act_num);
// return value is a score, representing the number of self-check tests that were failed
// return value could then be considered a 'confidence value' in the absence of the sensor
// 0: sensor believed to exist and is working (ALL tests passed)
// 1 - 5: number of tests that failed, the higher the value, the more confident that sensor is not connected



#ifdef __cplusplus
}
#endif

#endif
