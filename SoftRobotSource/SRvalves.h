// SRvalves.h

#ifndef __SRVALVES_H__
#define __SRVALVES_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif

#define VALVE_1_PORT		PORTF
#define VALVE_11_PIN_bm		PIN7_bm
#define VALVE_12_PIN_bm		PIN6_bm

#define VALVE_2_PORT		PORTC
#define VALVE_21_PIN_bm		PIN0_bm
#define VALVE_22_PIN_bm		PIN1_bm

#define VALVE_3_PORT		PORTC
#define VALVE_31_PIN_bm		PIN5_bm
#define VALVE_32_PIN_bm		PIN4_bm

#define VALVE_4_PORT		PORTD
#define VALVE_41_PIN_bm		PIN6_bm
#define VALVE_42_PIN_bm		PIN7_bm

#define VALVE_5_PORT		PORTE
#define VALVE_51_PIN_bm		PIN2_bm
#define VALVE_52_PIN_bm		PIN3_bm

#define VALVE_6_PORT		PORTF
#define VALVE_61_PIN_bm		PIN4_bm
#define VALVE_62_PIN_bm		PIN5_bm

// CONVENTIONS:
// Indexing all valves/actuators begins with 1
// For all functions in this library, act_num = {1, ... 6}
// This is also consistent with the convention in SRpressure_control.c/h


/* INITIALIZATION */

void valves_init(void);

/* USER ACCESSIBLE FUNCTIONS */

// open the fill valve for the given actuator
void valve_fill(uint8_t act_num);

// open the fill valve for the given actuator
void valve_vent(uint8_t act_num);

// close both valves for the given actuator
void valve_hold(uint8_t act_num);

// open both valves for the given actuator
void valve_thru_open(uint8_t act_num);

// check the status of the fill valve for the given actuator
uint8_t fill_valve_is_open(uint8_t act_num);

// check the status of the vent valve for the given actuator
uint8_t vent_valve_is_open(uint8_t act_num);

// open the fill valve for all actuators
void valve_fill_ALL(void);

// open the vent valve for all actuators
void valve_vent_ALL(void);

// close both valves for all actuators
void valve_hold_ALL(void);

// open both valves for all actuators
void valve_thru_open_ALL(void);


/* INTERNAL FUNCTIONS */

void check_open_valves(void);



#ifdef __cplusplus
}
#endif

#endif