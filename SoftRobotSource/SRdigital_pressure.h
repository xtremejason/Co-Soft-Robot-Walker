//SRdigital_pressure.h

#ifndef __SRDIGITAL_PRESSURE_H__
#define __SRDIGITAL_PRESSURE_H__


#include <avr/io.h>

#include <stdio.h> // used for stdout

#include <util/delay.h> // used?

#include "math.h"

#include "software_clock.h"

#include "SRdigital_pressure.h"	// for debugging

#ifdef __cplusplus
extern "C"
{
#endif

// we are using Measurement Specialties MS4525DO Temperature and Pressure Sensor
#define DPS_ADDRESS			0x28	// this is a standard 7-BIT address!


// Library written for Measurement Specialties part # 5425D0-DS5AIO15DP
// options:
// DS => 'Dual Sideport'
// 5 => Vsupply = 5.0 V				(note: 3 => Vsupply = 3.3 V)
// A => output type = 10% to 90%	(note: B => output type = 5% to 95%)
// I => interface = I2C addr 0x28	(note: J => I2C addr 0x36, K => I2C addr 0x46)

// user settable defines:
#define DPS_DATA_EXPIRATION_TIME_ms	100 // data is fresh for at least this long (range: ? to 59999)


/* INITIALIZATION */

// initialize digital pressure controller
uint8_t init_DPS(void);
// returns FALSE if digital pressure sensor was not found


/* USER ACCESSIBLE FUNCTIONS */

uint8_t check_p_sensor_exists(void);	// return: 1 if exist, 0 if not

// read and print data from the pressure sensor
float get_digital_pressure(void);

// read and temperature data from the pressure sensor
float get_air_temp(void);

uint8_t digital_pressure_data_is_stale(void);

/* PRIVATE FUNCTIONS */

float get_digital_pressure_lastread(void);

void DPS_get_bits(uint8_t *status_bits, uint16_t *pressure_bits, uint16_t *temperature_bits);

float DPS_pressure_conversion(uint16_t pressure_bits);

float DPS_temperature_conversion(uint16_t temperature_bits);

uint8_t DPS_status_conversion(uint8_t status_bits);

#ifdef __cplusplus
}
#endif

#endif