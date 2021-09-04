// SRexternal_device.c

#include <avr/io.h>
#include <stdio.h>

#include "SRexternal_device.h"

#define EXTERNAL_DEVICE_DEBUG_LEVEL 0

void external_device_init()
{
	// Pins as output
	EXTERNAL_DEVICE_PORT.DIRSET = EXTERNAL_DEVICE_PIN_bm;
}

void external_device_on()
{
	EXTERNAL_DEVICE_PORT.OUTSET = EXTERNAL_DEVICE_PIN_bm;
}

void external_device_off()
{
	EXTERNAL_DEVICE_PORT.OUTCLR = EXTERNAL_DEVICE_PIN_bm;
}

uint8_t external_device_is_powered()
{
	if(EXTERNAL_DEVICE_DEBUG_LEVEL > 0)
		printf("EXT DEV PIN: %u\r\n", EXTERNAL_DEVICE_PORT.OUT & EXTERNAL_DEVICE_PIN_bm);

	if((EXTERNAL_DEVICE_PORT.OUT & EXTERNAL_DEVICE_PIN_bm) > 0)
		return 1;
	else
		return 0;
}