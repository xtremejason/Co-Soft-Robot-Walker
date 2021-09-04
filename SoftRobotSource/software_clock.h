// software_clock.h

#ifndef __SOFTWARE_CLOCK_H__
#define __SOFTWARE_CLOCK_H__

#include <avr/io.h>
#include <util/atomic.h> // for ATOMIC_RESTORESTATE

#ifdef __cplusplus
extern "C"
{
#endif

// user settable parameters:
#define NUM_MISC_TIMERS			 4 // currently set to 4, but could have a timer for each actuator (or more!)
#define MAIN_LOOP_TIMEOUT_ms	40 // time allotted to each pass through the 'main' loop

/* USER ACCESSIBLE FUNCTIONS */

float get_main_loop_period_s(void);	// returns MAIN_LOOP_TIMEOUT_ms/1000 [s]

float get_main_loop_frequency_Hz(void);	// returns (MAIN_LOOP_TIMEOUT_ms/1000)^(-1) [Hz]

// MAIN LOOP TIMER // -------------------------------------------------------
// should ONLY be called by the main loop to set its own countdown timer
void set_main_countdown_timer(uint16_t future_ms);

// check to see if the main loop countdown timer has expired
uint8_t main_countdown_timer_expired(void);

// request permission to overrun the main loop timer without penalty
// this is a one-time only permission, must call each loop that overtime is requested
void request_overtime_permission(void);

// MISCELLANEOUS TIMERS // -------------------------------------------------------
// set a miscellaneous timer, timer_num range: [1, 2, ... NUM_MISC_TIMERS]
void set_misc_countdown_timer(uint8_t timer_num, uint16_t future_ms);

// check if ANY of the misc timers are expired, boolean
// 1 = there exists an expired timer, 0 = none are expired or are set
// user MUST call get_expired_timer_number() to check which timer this is associated with!
uint8_t misc_countdown_timer_expired(void);

// get the timer number associated with the most recent expired timer, 0 if none
uint8_t get_expired_timer_number(void);

// CLOCK FUNCTIONS // -------------------------------------------------------
// current milliseconds on the clock, note there is no seconds-clock
// returns 0 - 59999
uint16_t get_milliseconds(void);

// current minutes on the clock
uint8_t get_minutes(void);

// boolean toggle for automatically printing the time (e.g. in an interrupt)
void deactivate_print_time(void);
void activate_print_time(void);

// print the current time in mm:ss format
void print_time(void);

// GENERAL TIMER FUNCTIONS // -------------------------------------------------------
// computes the difference in milliseconds between last_time_ms and this_time_ms
// compensates for clock rollover in case last_time_ms was in the 'last minute'
uint16_t compute_tdiff_ms(uint16_t last_time_ms, uint16_t this_time_ms);


/* INTERNAL FUNCTIONS */

// add 1 ms to the clock, if ms == 60000 calls increment_minutes()
void increment_milliseconds(void);

// add 1 minute to the clock
void increment_minutes(void);


/* INITIALIZATION */

// no init - no hardware


#ifdef __cplusplus
}
#endif

#endif