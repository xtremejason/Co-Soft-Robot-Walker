//SRtouch_control.h

#ifndef __SR_TOUCH_CONT_H__
#define __SR_TOUCH_CONT_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif


#define CAP_VALUE_TYPE_ADC			0
#define CAP_VALUE_TYPE_CAPACITANCE	1
#define CAP_VALUE_TYPE_PROXIMITY	2
#define CAP_VALUE_TYPE_DELTA_ADC	3

// Global Constants



//#define SENSOR_STRIP_VERSION	1	// squares
#define SENSOR_STRIP_VERSION	2	// rectangles
//#define SENSOR_STRIP_VERSION	3	// rectangles
//#define SENSOR_STRIP_VERSION	4	// triangles
//#define SENSOR_STRIP_VERSION	5	// triangles
//#define SENSOR_STRIP_VERSION	6	// squares
//#define SENSOR_STRIP_VERSION	101	// prototype 1, no PCB, 6 electrodes
//#define SENSOR_STRIP_VERSION	102	// prototype 2, no PCB, 8 electrodes (now broken)


#define TOUCH_ACTUATOR_NUM		1	// position of the actuator on the robot

#define FILL_TOUCHPAD			8
#define VENT_TOUCHPAD			7

#define TOUCH_CONTROL_ON		0	// do not act on touches
//#define TOUCH_CONTROL_ON		1	// use touch input as control

#define CONTACT_THRESHOLD_PCT	25	// arbitrary, needs analysis
//#define CONTACT_THRESHOLD_PCT	1	// proximity?


#define MAX_NUM_TOUCH_PADS		8	// maximum possible number of touch pads, for memory allocation elsewhere


/* PAD-LEVEL FUNCTIONS */

uint8_t get_cap_values_allpads(float *values, uint8_t type, uint8_t baseline);
// returns the number of values in the 'return' array

// for DEBUG use:
void print_cap_values_allpads(uint8_t type, uint8_t baseline);

float get_cap_value_1pad(uint8_t touch_pad_number, uint8_t type, uint8_t baseline);
// touch_pad_number: integer >= 1, number of the touch pad
// type: CAP_VALUE_TYPE_.. enum in #defines above
// baseline: boolean, 1 = return baseline values, 0 = return current values


/* ACTUATOR-LEVEL FUNCTIONS */

// return by pointer: a bit-flag representation of which touch pads have been touched since
// the last time this function has been called	[note: 0b00000001 represents pad-1]
// (e.g. no-touch -> touch transition and touch -> no-touch transition)
// return by uint8_t: a logical-OR for any transition that occurred (treated as a boolean)
uint8_t get_touch_status_change(uint8_t *touch_change, uint8_t *untouch_change);

// returns a bit-flag representation of which touch pads are in 'contact' with the user
// currently the touch threshold is hard coded, though could be passed as the parameter
// note: 0b00000001 represents pad-1
uint8_t get_touch_pads_in_contact(void);

// simple check of number of touch pads (being used), <= MAX_NUM_TOUCH_PADS
uint8_t get_num_touch_pads(void);


/* MAIN-LOOP FUNCTION (CONTROLLER) */

void touch_controller(uint8_t touch_events, uint8_t untouch_events);


/* INITIALIZATION */

void init_touch_control(void);

void define_electrode_strip_connections(uint8_t strip_number);

/* INTERNAL FUNCTIONS */


float reshape_C_ADC(uint16_t ADC_value, uint16_t baseline_value, uint8_t type);


#ifdef __cplusplus
}
#endif

#endif