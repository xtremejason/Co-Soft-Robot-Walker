// RGB_LED.h

#ifndef __SRSTATUS_LED_H__
#define __SRSTATUS_LED_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#include <stdio.h> // used for stdout (e.g. printf)

#ifdef __cplusplus
extern "C"
{
#endif

#define LED_1_PORT 				PORTD		
#define LED_1_PIN_bm			PIN5_bm		
#define LED_1_TC				TCD1			// on TCD1-CCB
#define LED_1_CCREG				CCB				// on Clock-Compare-B
#define LED_1_CC_ENABLE_bm		TC1_CCBEN_bm	// on Clock-Compare-B
#define LED_1_CCBUF				CCBBUF

#define LED_2_PORT 				PORTD		
#define LED_2_PIN_bm			PIN0_bm		
#define LED_2_TC				TCD0			// on TCD0-CCA
#define LED_2_CCREG				CCA				// on Clock-Compare-A
#define LED_2_CC_ENABLE_bm		TC1_CCAEN_bm	// on Clock-Compare-A
#define LED_2_CCBUF				CCABUF


// user configurable defines:
#define LED_SATURATION_OFF		  0	// out of 255
#define LED_SATURATION_NORMAL	100	// out of 255
#define LED_SATURATION_HIGH		255	// out of 255


/* INITIALIZATION */

// sets LEDs to run off PWM control
void status_LED_init(void);


/* USER ACCESSIBLE FUNCTIONS */

void toggle_LED(uint8_t LED_num);

// get intensity for LED 1 or 2
uint8_t get_LED(uint8_t LED_num);

// set intensity for LED 1 or 2
void set_LED(uint8_t LED_num, uint8_t saturation);
// saturation is any value between 0 (off) and 255 (full saturation), inclusive



#ifdef __cplusplus
}
#endif

#endif