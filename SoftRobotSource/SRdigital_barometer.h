//SRdigital_barometer.h

#ifndef __SRDIGITAL_BAROMETER_H__
#define __SRDIGITAL_BAROMETER_H__


#include <avr/io.h>

#include <stdio.h> // used for stdout



#ifdef __cplusplus
extern "C"
{
#endif


// user settable defines:
#define BPS_DATA_EXPIRATION_TIME_ms	100		// note: in HI-RES mode conversions take >= 16.44 ms


/* USER ACCESSIBLE FUNCTIONS */

// boolean check to see if the device is present on the I2C bus
uint8_t check_BPS_exists(void);
// return 1 if sensor exists on I2C bus

// the natural unit of pressure for this sensor is millibar, resolution 0.01 mbar
float get_BPS_pressure_mbar(void);		

// the natural unit of temperature for this sensor is degrees Celsius, resolution 0.01 °C
float get_BPS_temperatue_C(void);	

// boolean check if the last taken data is still fresh
uint8_t BPS_data_is_stale(void);
// returns 1 if pressure/temp data is more than BPS_DATA_EXPIRATION_TIME_ms old


/* INTERNAL FUNCTIONS - HIGH LEVEL */

void BPS_read_convert_data(void);

void BPS_read_convert_D1(void);

void BPS_read_convert_D2(void);

void BPS_read_calibration_PROM(void);


/* INTERNAL FUNCTIONS - LOW LEVEL */

void BPS_reset_sequence(void);

/* DEBUG FUNCTIONS */

//void BPS_read_TEST(void);

void BPS_read_convert_D1_TEST(void);

void BPS_read_convert_D2_TEST(void);

void BPS_read_calibration_PROM_TEST(void);

void BPS_print_calibration_PROM(void);



/* INITIALIZATION */

// initialize barometric pressure sensor
uint8_t init_BPS(void);


#ifdef __cplusplus
}
#endif

#endif