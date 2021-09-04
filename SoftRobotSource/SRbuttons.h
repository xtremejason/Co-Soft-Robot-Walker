//SRbuttons.h

#ifndef __SRBUTTONS_H__
#define __SRBUTTONS_H__

#include <avr/io.h>
#include <stdio.h> // used for stdout

#include <avr/interrupt.h>


#include "software_clock.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BUTTON_A	1
#define BUTTON_B	2


#define BUTTON_EVENT_A_DOWN		1
#define BUTTON_EVENT_A_UP		2
#define BUTTON_EVENT_B_DOWN		3
#define BUTTON_EVENT_B_UP		4
#define BUTTON_EVENT_NONE		0


/* USER ACCESSIBLE FUNCTIONS */

// check the current pressed-status of either of the peripheral buttons
uint8_t read_button(uint8_t b_num);
// b_num:
// BUTTON_B = (blue wire) = C6
// BUTTON_A = (green wire) = C7
//returns:
// 0 = not pressed
// 1 = pressed

// get the current (most recent) event status (if any) of either of the peripheral buttons
uint8_t get_button_press_event(void);
// returns: BUTTON_EVENT_type (see #defines above)


/* INITIALIZATION */

// initialize button controller
void init_buttons(void);


#ifdef __cplusplus
}
#endif

#endif