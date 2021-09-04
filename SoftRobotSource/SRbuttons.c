// SRbuttons.c

#include "SRbuttons.h"

#include <util/delay.h>



uint16_t button_last_event_time_ms = 0;

#define BUTTON_DEBOUNCE_TIME_ms		50

// bit masks for the buttons port (portC)
#define BUTTON_A_bm	0b01000000
#define BUTTON_B_bm	0b10000000
// note(!): these bit masks are chosen specifically to cover the portC register,
// however they are also used with the private buttons variables (flags):

// private buttons variables:
uint8_t buttons_down = 0;
uint8_t buttons_change = 0;

#define BUTTON_DEBUG_PRINT_MODE 	0	// no printing
//#define BUTTON_DEBUG_PRINT_MODE 	1	// print button UP/DOWN events

/* INITIALIZATION */

void init_buttons()
{	
	// note: the buttons are connected IN PLACE of serial port NODE 1
	// the pins for this are RX1 = portCpin6 (pin22 of MCU)
	// the pins for this are TX1 = portCpin7 (pin23 of MCU)
	
	// DISABLE THE PORTS SERIAL FUNCTIONS:

	PORTC.DIRCLR = 0b11000000;
	
	// set up interrupt
	
	//PORTB.PIN2CTRL |= 0b01000000;	// inverts the logic on portBpin2

	//PORTB.PIN2CTRL |= 0b00010000;	// enables pull-down resistor on portBpin2
	PORTC.PIN6CTRL |= 0b00011000;	// enables pull-up resistor on portCpin6
	PORTC.PIN7CTRL |= 0b00011000;	// enables pull-up resistor on portCpin7
	
	//PORTC.PIN6CTRL |= 0b00000010;	// trigger on falling-edge 
	//PORTC.PIN6CTRL |= 0b00000001;	// trigger on rising-edge
	PORTC.PIN6CTRL |= 0b00000000;	// trigger on both edges
	
	PORTC.INT0MASK = 0b11000000;	// sets portC's interrupt 0 to be tied to pin 6 & 7
	
	PORTC.INTCTRL  |= PORT_INT0LVL_LO_gc;	// enables INT0 at LOW-priority, PORT_INT0LVL_LO_gc = 0b00000001
}

/* USER ACCESSIBLE FUNCTIONS */

uint8_t read_button(uint8_t b_num)
{
	uint8_t port_pins;
	uint8_t ret_value;
	
	port_pins = PORTC.IN;
	
	if(b_num == BUTTON_A)
	{
		if(port_pins & BUTTON_A_bm)
			ret_value = 1;
		else
			ret_value = 0;
	}
	
	else if(b_num == BUTTON_B)
	{
		if(port_pins & BUTTON_B_bm)
			ret_value = 1;
		else
			ret_value = 0;
	}
	
	else
	{
		printf("WRONG CHOICE\r\n");
		ret_value = 0;
	}
	
	return ret_value;
}


uint8_t get_button_press_event(void)
{
	uint8_t button_event;
	
	if(buttons_change & BUTTON_A_bm)
	{
		if(buttons_change & buttons_down)
		{	// positive down event
			buttons_change &=~ BUTTON_A_bm;
			if(BUTTON_DEBUG_PRINT_MODE){printf("A down\r\n");}
			button_event = BUTTON_EVENT_A_DOWN;
		}
		if(buttons_change & ~buttons_down)
		{	// positive up event
			buttons_change &=~ BUTTON_A_bm;
			if(BUTTON_DEBUG_PRINT_MODE){printf("A up\r\n");}
			button_event = BUTTON_EVENT_A_UP;
		}
	}
		
	else if(buttons_change & BUTTON_B_bm)
	{
		if(buttons_change & buttons_down)
		{	// positive down event
			buttons_change &=~ BUTTON_B_bm;
			if(BUTTON_DEBUG_PRINT_MODE){printf("B down\r\n");}
			button_event = BUTTON_EVENT_B_DOWN;
		}
		if(buttons_change & ~buttons_down)
		{	// positive up event
			buttons_change &=~ BUTTON_B_bm;
			if(BUTTON_DEBUG_PRINT_MODE){printf("B up\r\n");}
			button_event = BUTTON_EVENT_B_UP;
		}
	}

	else
		button_event = BUTTON_EVENT_NONE;

	return button_event;
}


//interrupt function for buttons
ISR(PORTC_INT0_vect)
{		
	uint16_t time_ms = get_milliseconds();
	
	uint16_t t_diff = compute_tdiff_ms(button_last_event_time_ms, time_ms);
	
	if(t_diff > BUTTON_DEBOUNCE_TIME_ms)
	{ // if enough time has passed since the last check to expect bounces to be over
		if(read_button(BUTTON_A))
		{ // if A is down..
			if((buttons_down & BUTTON_A_bm) == 0)
			{ // .. and A is expected to be up, then a DOWN event occurred
				buttons_down |= BUTTON_A_bm;
				//printf("EVENT A down\r\n");
	
				buttons_change |= BUTTON_A_bm;

				button_last_event_time_ms = time_ms;	// global assignment
			}
		}
		else
		{ // if A is up..
			if((buttons_down & BUTTON_A_bm) == BUTTON_A_bm)
			{ // .. and A is expected to be down, then an UP event occurred
				buttons_down &=~ BUTTON_A_bm;
				//printf("EVENT A up\r\n");

				buttons_change |= BUTTON_A_bm;
				
				button_last_event_time_ms = time_ms;
			}
		}

		if(read_button(BUTTON_B))
		{ // if B is down..

			if((buttons_down & BUTTON_B_bm) == 0)
			{ // .. and B is expected to be up, then a DOWN event occurred
				buttons_down |= BUTTON_B_bm;
				//printf("EVENT B down\r\n");

				buttons_change |= BUTTON_B_bm;
				
				button_last_event_time_ms = time_ms;
			}
		}
		else
		{ // if B is up..

			if((buttons_down & BUTTON_B_bm) == BUTTON_B_bm)
			{ // .. and B is expected to be down, then an UP event occurred
				buttons_down &=~ BUTTON_B_bm;
				//printf("EVENT B up\r\n");

				buttons_change |= BUTTON_B_bm;
				
				button_last_event_time_ms = time_ms;
			}
		}
	
	}	// end if(t_diff > BUTTON_DEBOUNCE_TIME_ms)
}
