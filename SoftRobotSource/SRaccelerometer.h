//SRaccelerometer.h

#ifndef __SRACCELEROMETER_H__
#define __SRACCELEROMETER_H__


#include <avr/io.h>

#include <stdio.h> // used for stdout

//#include "math.h"

#ifdef __cplusplus
extern "C"
{
#endif


// we are using the ADXL362 (Analog Devices 3-Axis MEMS Accelerometer)
// This is an SPI device!


/* USER ACCESSIBLE FUNCTIONS */

uint8_t check_accel_exists(void);	// return: 1 if exist, 0 if not

// read and print all measured data from the accelerometer
void accel_status(void);


/* INTERNAL FUNCTIONS */

// read a register (8bit) via SPI
uint8_t accel_read(uint8_t read_address);

// write 8bit data into registers via SPI
void accel_write(uint8_t addr, uint8_t data);

// convert raw data into decimal with sign and units of g's
float accel_data_to_number(uint8_t axis_data);


//prints tap to stdout
void tap(uint8_t tilt);

//prints shake to stdout
void shake(uint8_t tilt);


/* INITIALIZATION */

// initialize accelerometer controller
uint8_t init_accel(void);



#ifdef __cplusplus
}
#endif

#endif