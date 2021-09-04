//SR9axis_IMU.h

#ifndef __SR9AXIS_IMU_H__
#define __SR9AXIS_IMU_H__


#include <avr/io.h>

#include <stdio.h> // used for stdout

#ifdef __cplusplus
extern "C"
{
#endif

// Implementation for BMX055 9-axis IMU (Accelerometer, Gyroscope, Magnetometer)



// these are (standard) 7-BIT I2C addresses:
#define BMX055_ACCEL_1_ADDRESS		0x18
#define BMX055_GYRO_1_ADDRESS 		0x68
#define BMX055_MAG_1_ADDRESS		0x10

#define BMX055_ACCEL_2_ADDRESS		0x19
#define BMX055_GYRO_2_ADDRESS		0x69
#define BMX055_MAG_2_ADDRESS		0x11



// accelerometer registers [pg. 49 in the datasheet]
#define ACCD_X_LSB		0X02
#define ACCD_X_MSB		0X03
#define ACCD_Y_LSB		0X04
#define ACCD_Y_MSB		0X05
#define ACCD_Z_LSB		0X06
#define ACCD_Z_MSB		0X07
#define ACC_SOFTRESET	0x14	// write 0xB6 to trigger the reset

// gyroscope registers [pg. 92 in the datasheet]
#define RATE_X_LSB		0X02
#define RATE_X_MSB		0X03
#define RATE_Y_LSB		0X04
#define RATE_Y_MSB		0X05
#define RATE_Z_LSB		0X06
#define RATE_Z_MSB		0X07
#define GYR_SOFTRESET	0x14	// write 0xB6 to trigger the reset

// magnetometer registers [pg. 130 in the datasheet]
#define M_DATA_X_LSB	0X42
#define M_DATA_X_MSB	0X43
#define M_DATA_Y_LSB	0X44
#define M_DATA_Y_MSB	0X45
#define M_DATA_Z_LSB	0X46
#define M_DATA_Z_MSB	0X47
#define MAG_PWR_CTRL	0x4B	// default 0x01, write 0x82 to trigger the reset
#define MAG_MODE_CTRL	0x4C	// default 0x06, write 0x00 to set to normal mode (wake sleep)

// note: bits are referred to by number in range 0...7

// TODO! (TODO) : Needs a reset function

/* USER ACCESSIBLE FUNCTIONS - ACCELEROMETER */

uint8_t check_IMU_accel_sensor_exists(uint8_t IMU_num);

uint8_t verify_IMU_accel_existance(uint8_t IMU_num);

uint8_t get_9axis_accel_bytes(int16_t* accel, uint8_t IMU_num);
// 2 bytes for each axis, value range: -2048 to 2047 (in 12-bit 2's complement)
// return 1 if new accel array was collected

uint8_t get_9axis_accel_g(float *accel, uint8_t include_norm, uint8_t IMU_num);
// units: g (assumed to be 9.80 m/s^2)
// return 1 if new accel array was collected

uint8_t get_9axis_accel_SI(float *accel, uint8_t IMU_num);

float get_9axis_accel_resolution(uint8_t IMU_num);

// debugging, read and print contents of every single accelerometer register
void register_read_print_9axis_accel(uint8_t IMU_num);

/* PRIVATE FUNCTIONS - ACCELEROMETER */

uint8_t address_of_IMU_accel(uint8_t IMU_num);



/* USER ACCESSIBLE FUNCTIONS - GYROSCOPE */

uint8_t check_IMU_gyro_sensor_exists(uint8_t IMU_num);

uint8_t verify_IMU_gyro_existance(uint8_t IMU_num);

uint8_t get_9axis_gyro_bytes(int16_t* gyro, uint8_t IMU_num);
// 2 bytes for each axis (in 16-bit 2's complement)
// returns 1 if new gyro array was collected

uint8_t get_9axis_gyro_deg(float *gyro, uint8_t IMU_num);
// units: degrees/second
// (note: ~ 57.3º/radian)
// (note: ø can be used to print º to the terminal)
// returns 1 if new gyro array was collected

uint8_t get_9axis_gyro_SIrad(float *gyro, uint8_t IMU_num);
// units: radians/second
// returns 1 if new gyro array was collected

void set_9axis_gyro_resolution(uint8_t bits, uint8_t IMU_num);

float get_9axis_gyro_resolution(uint8_t IMU_num);

void double_9axis_gyro_resolution(uint8_t IMU_num);

// debugging, read and print contents of every single gyroscope register
void register_read_print_9axis_gyro(uint8_t IMU_num);

uint8_t address_of_IMU_gyro(uint8_t IMU_num);




/* USER ACCESSIBLE FUNCTIONS - MAGNETOMETER */

uint8_t check_IMU_mag_sensor_exists(uint8_t IMU_num);

uint8_t verify_IMU_mag_existance(uint8_t IMU_num);

//uint8_t get_9axis_mag_bytes(int16_t* mag);
uint8_t get_9axis_mag_bytes(int16_t* mag, uint8_t IMU_num);
// 2 bytes for each axis (in 16-bit 2's complement)
// returns 1 if new mag array was collected

uint8_t get_9axis_mag_uT(float *mag, uint8_t IMU_num);
// units: 
// returns 1 if new mag array was collected

float get_9axis_mag_resolution(uint8_t IMU_num);

// debugging, read and print contents of every single magnetometer register
void register_read_print_9axis_mag(uint8_t IMU_num);

void single_register_read_print_9axis_mag(uint8_t reg, uint8_t IMU_num);

/* PRIVATE FUNCTIONS - MAGNETOMETER */

uint8_t address_of_IMU_mag(uint8_t IMU_num);

void load_mag_fake_bytes(uint8_t* mag_bytes);




/* INITIALIZATION */

uint8_t init_9axis_accel(void);

uint8_t init_9axis_gyro(void);

uint8_t init_9axis_mag(void);


#ifdef __cplusplus
}
#endif

#endif