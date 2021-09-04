// software_clock.c
// file history: this file was copied then modified from time_manager.cpp (hex blocks)

#include "software_clock.h"

#include "SRmisc_functions.h"	// for halt_program()
#include <stdio.h>				// for printf

// MAIN COUNTDOWN TIMER (controls main loop!)
uint16_t main_countdown_timer_goal;
uint8_t main_countdown_timer_mustroll;
uint8_t main_countdown_timer_ticking = 0;
uint8_t main_countdown_timer_overtime_granted = 0;	// boolean

// MISC COUNTDOWN TIMERS
uint16_t misc_countdown_timer_goal[NUM_MISC_TIMERS];
uint8_t misc_countdown_timer_mustroll[NUM_MISC_TIMERS];			// boolean, set if the timer goal will occur in the NEXT minute
uint8_t misc_countdown_timer_ticking[NUM_MISC_TIMERS] = {0};	// boolean, timers that are not yet expired are considered 'ticking'
uint8_t misc_countdown_timer_set[NUM_MISC_TIMERS] = {0};		// boolean, turn this off only when which timer that expired has been reported
uint8_t misc_expired_notified = 0;								// boolean

// TIME VARIABLES
uint16_t milliseconds = 0;
uint8_t minutes = 0;

// RTC
//uint16_t rtc_seconds = 0;

// CONTROL VARIABLES
uint8_t allow_print_time = 0;	// boolean


/* USER ACCESSIBLE FUNCTIONS */

float get_main_loop_period_s(void)
{	
	//float period;
	//period = MAIN_LOOP_TIMEOUT_ms/1000.0;
	//return period;

	return MAIN_LOOP_TIMEOUT_ms/1000.0;
}

float get_main_loop_frequency_Hz(void)
{
	return 1000.0/MAIN_LOOP_TIMEOUT_ms;
}

// MAIN LOOP TIMER // -------------------------------------------------------
void set_main_countdown_timer(uint16_t future_ms)
{
	main_countdown_timer_goal = milliseconds + future_ms;

	// check to see if the main timer will need to rollover (e.g. set time is going to happen in the NEXT minute)
	if(main_countdown_timer_goal > 60000)
	{
		main_countdown_timer_goal -= 60000;
		main_countdown_timer_mustroll = 1;		// currently, only 1 rollover is permitted,
	}

	main_countdown_timer_ticking = 1;	// set boolean
}

uint8_t main_countdown_timer_expired(void)
{
	if(main_countdown_timer_ticking)
		return 0;	// FALSE
	else if(main_countdown_timer_overtime_granted)
	{	// overtime permission was granted, let this one go
		main_countdown_timer_overtime_granted = 0;	// revoke permission for next timer cycle
		return 0;
	}
	
	return 1;	// TRUE
}

void request_overtime_permission(void)
{
	main_countdown_timer_overtime_granted = 1;
}

// MISCELLANEOUS TIMERS // -------------------------------------------------------
void set_misc_countdown_timer(uint8_t timer_num, uint16_t future_ms)
{
	uint8_t timer_index;  // range: 0 - (NUM_MISC_TIMERS-1)
	
	if(timer_num == 0)
	{
		printf("ERROR: no timer 0\r\n");
		halt_program();
		return;
	}
	
	// timer index is 1-less than timer number (timer numbers begin with 1)
	timer_index = timer_num - 1;
	
	misc_countdown_timer_goal[timer_index] = milliseconds + future_ms;

	// check to see if the misc timer will need to rollover (e.g. set time is going to happen in the NEXT minute)
	if(misc_countdown_timer_goal[timer_index] > 60000)
	{
		misc_countdown_timer_goal[timer_index] -= 60000;
		misc_countdown_timer_mustroll[timer_index] = 1;		// currently, only 1 rollover is permitted,
	}

	misc_countdown_timer_set[timer_index] = 1;
	misc_countdown_timer_ticking[timer_index] = 1;
}

uint8_t misc_countdown_timer_expired(void)
{
	if(misc_expired_notified)
		printf("ERROR: already notified of expired timer\r\n");
	
	// if a timer is ticking and set, then it is still counting down
	// if a timer is set but not ticking, then it has expired and should be reported
	// if a timer is not set and not ticking, then it is not in use
	// if a timer is ticking but not set, this should not happen!	
	for(uint8_t timer_index = 0; timer_index < NUM_MISC_TIMERS; timer_index++)
	{
		if( (misc_countdown_timer_set[timer_index])&&(!misc_countdown_timer_ticking[timer_index]) )
		{	// this timer should be reported as expired
			misc_expired_notified = 1;	// the user has been notified that a timer has expired

			// note: do not un-set the timer until the user checks which one has expired
			return 1;	// it is up to the user to check which timer has expired
		}
		
		if( (!misc_countdown_timer_set[timer_index])&&(misc_countdown_timer_ticking[timer_index]) )
		{	// ticking but not set, this should not happen
			printf("ERROR: timer %u ticking but not set\r\n", timer_index + 1);
		}
	}
	
	return 0;
}

uint8_t get_expired_timer_number(void)
{
	uint8_t timer_num;
	
	if(!misc_expired_notified)
	{
		printf("ERROR: no timer has expired\r\n");
		return 0;
	}
	
	for(uint8_t timer_index = 0; timer_index < NUM_MISC_TIMERS; timer_index++)
	{
		if( (misc_countdown_timer_set[timer_index])&&(!misc_countdown_timer_ticking[timer_index]) )
		{	// this timer should be reported as expired
			timer_num = timer_index + 1;
			misc_countdown_timer_set[timer_index] = 0; // un-set this timer and report it has expired
			misc_expired_notified = 0;	// un-set before checking for another expired timer
			
			return timer_num;
		}
	}
	
	return 0;	// this should not happen
}

// CLOCK FUNCTIONS // -------------------------------------------------------
uint16_t get_milliseconds(void)
{
	return milliseconds;
}

uint8_t get_minutes(void)
{
	return minutes;
}

void deactivate_print_time(void)
{
	allow_print_time = 0;
}

void activate_print_time(void)
{
	allow_print_time = 1;
}

void print_time(void)
{
	uint8_t seconds = milliseconds/1000;
	printf("%u:%02u\r\n",minutes,seconds);
}

// GENERAL TIMER FUNCTIONS // -------------------------------------------------------
uint16_t compute_tdiff_ms(uint16_t last_time_ms, uint16_t this_time_ms)
{
	uint16_t t_diff;
	
	if(last_time_ms > this_time_ms)
	{	// then clock rollover occurred
		t_diff = (60000 - last_time_ms) + this_time_ms;
	}
	else
	{
		t_diff = this_time_ms - last_time_ms;
	}
	
	return t_diff;
}


/* INTERNAL FUNCTIONS */

void increment_milliseconds(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) // Disable interrupts
	{
		milliseconds++;
		
		// check to SEE and SET if the main timer has expired:
		if((main_countdown_timer_ticking)&&(!main_countdown_timer_mustroll)&&(milliseconds >= main_countdown_timer_goal))
		main_countdown_timer_ticking = 0;
		
		// check to SEE and SET if the misc countdown timers have expired:
		for(uint8_t t = 0; t < NUM_MISC_TIMERS; t++)
		{
			if((misc_countdown_timer_ticking[t])&&(!misc_countdown_timer_mustroll[t])&&(milliseconds >= misc_countdown_timer_goal[t]))
			misc_countdown_timer_ticking[t] = 0;
		}

		// check for minute-rollover:
		if(milliseconds == 60000)
		{
			milliseconds = 0;
			increment_minutes();

			if(main_countdown_timer_mustroll)
				main_countdown_timer_mustroll--;		// sets to 0 if number of rollovers is 1

			for(uint8_t t = 0; t < NUM_MISC_TIMERS; t++)
			{
				if(misc_countdown_timer_mustroll[t])
				misc_countdown_timer_mustroll[t]--;		// sets to 0 if number of rollovers is 1
			}
		}
	} // end ATOMIC_BLOCK

	if(milliseconds%1000 == 0)
	{	// clock functions ONLY should go here
		// this loop happens once a second
		if(allow_print_time)
		print_time();
	}
}

void increment_minutes(void)
{
	minutes++;	// rollover not implemented yet (no hours)
}


/*	RTC CODE - NOT ENABLED
void RTC_init_new()
{
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;	// ref: doc8331.pdf, pg 92

	//CLK_RTCSRC_RCOSC_gc = (0x02<<1),  // 1.024 kHz from internal 32.768 kHz RC oscillator
	//#define CLK_RTCEN_bm  0x01  // Clock Source Enable bit mask.

	RTC.INTCTRL = RTC_OVFINTLVL_MED_gc;              // med level overflow interrupt to increment the epoch counter
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.PER = 0xFFFF;
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.CNT = 0;
}

ISR( RTC_OVF_vect )
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) // Disable interrupts
	{
		rtc_seconds++; // don't know why this is seconds, as this called once a minute
		// TODO: find out what settings are, such that this is called once a minute
	}

	if(allow_print_time)
	{
		// this goes off once a minute
		printf_P(PSTR("tock %i\n\r"),rtc_seconds);
	}
}
*/