//SRaccel_gyro.c

// Implementation for BMI160 6-axis IMU Accelerometer Gyroscope

#include "SR9axis_IMU.h"

#include <util/delay.h>
#include <stdio.h>			// for printf

#include "SoftRobotI2C.h"
#include "software_clock.h"
#include "SRmisc_functions.h"

#define IMU_A_DEBUG_MODE    	0		// no printing
//#define IMU_A_DEBUG_MODE  	1		// print acceleration			(floats)
//#define IMU_A_DEBUG_MODE  	2		// print intermediate results	(bytes)
//#define IMU_A_DEBUG_MODE  	3		// print I2C data stream

#define IMU_G_DEBUG_MODE    	0		// no printing
//#define IMU_G_DEBUG_MODE    	1		// print final conversion		(floats)
//#define IMU_G_DEBUG_MODE  	2		// print intermediate results	(bytes)
//#define IMU_G_DEBUG_MODE    	3		// print I2C data stream

#define IMU_M_DEBUG_MODE    	0		// no printing
//#define IMU_M_DEBUG_MODE    	1		// print final conversion		(floats)
//#define IMU_M_DEBUG_MODE    	2		// print intermediate results	(bytes)
//#define IMU_M_DEBUG_MODE    	3		// print I2C data stream

// TODO: include provision for there being 2 IMUs
uint8_t x9A_existance_confirmed[2] = {0,0};	// boolean, set in init
uint8_t x9G_existance_confirmed[2] = {0,0};	// boolean, set in init
uint8_t x9M_existance_confirmed[2] = {0,0};	// boolean, set in init


#define DEFAULT_GYRO_RESOLUTION		2000.0	// in degrees/second (per setting that gyro boots to)
float gyro_resolution[2];					// set to default in init


#define LITTLE_G		9.8	// m/s^2
	
/* USER ACCESSIBLE FUNCTIONS - ACCELEROMETER */

uint8_t check_IMU_accel_sensor_exists(uint8_t IMU_num)
{
	// sometimes single check would fail, so double check
	//I2C_check_device_exists(BMX055_ACCEL_1_ADDRESS);
	I2C_check_device_exists(address_of_IMU_accel(IMU_num));
	
	//return I2C_check_device_exists(BMX055_ACCEL_1_ADDRESS);
	return I2C_check_device_exists(address_of_IMU_accel(IMU_num));
}

uint8_t verify_IMU_accel_existance(uint8_t IMU_num)
{
	if(!x9A_existance_confirmed[IMU_num-1])
	{
		printf("BMX055-A-%u not found\r\n", IMU_num);
		_delay_ms(5);
	}
	// TODO: include provision for there being 2 IMUs  

	return x9A_existance_confirmed[IMU_num-1];
}

uint8_t get_9axis_accel_bytes(int16_t *accel, uint8_t IMU_num)
{
	// [timing measured with 100 kHz I2C clock, unmeasured with current 400 kHz clock]
	// this operation, without printing takes about 1 ms
	// sometimes (~ 10%) the IMU does I2C clock-stretching and the read takes around 2.75 ms

	uint8_t success = 0;	// assume fail until success is detected

	if((success=verify_IMU_accel_existance(IMU_num)))
	{
		uint8_t bytes[6];
		uint8_t i;
		uint16_t intermediate;

		I2C_read_bytes(address_of_IMU_accel(IMU_num), ACCD_X_LSB, bytes, 6);
		
		if(IMU_A_DEBUG_MODE > 2) for(i = 0; i < 6; i++)	printf("%u: %i\r\n", i, bytes[i]);

		// memory map:
		// ADDR		description		 7	6  5  4  3  2  1  0		[bit number]
		// 0x02		x-LSB			 3  2  1  0  x  x  x  N		N = new data 
		// 0x03		x-MSB			11 10  9  8  7  6  5  4

		for(i = 0; i < 3; i++)
		{
			intermediate = bytes[(2*i)+1];
			intermediate = intermediate<<4;
			intermediate += (bytes[2*i]>>4)&0x0F;

			// check new data flag, zero value means this data has been read already
			if(!(bytes[2*i]&0x01))
			{
				printf("x");
				success = 0;
			}

			accel[i] = twos_complement_conversion(intermediate, 11);
		}
	}

	if(IMU_A_DEBUG_MODE > 1)
	{
		printf("A: [%i, %i, %i]\r\n", accel[0], accel[1], accel[2]);
		_delay_ms(5);
	}
	
	return success;
}

uint8_t get_9axis_accel_g(float *accel, uint8_t include_norm, uint8_t IMU_num)
{
	int16_t vals[3];
	float vec_norm = 0;
	uint8_t i;
	uint8_t success;

	success = get_9axis_accel_bytes(vals, IMU_num);

	if(success)
	{
		for(i = 0; i < 3; i++)
		{
			// default range is +/- 2.0 g
			accel[i] = vals[i];				// int16_t -> float
			accel[i] = accel[i]/1024.0;		// 1024 = 2^10 = 1<<10
			vec_norm += accel[i]*accel[i];	// sum, for L2 norm of acceleration vectors
		}

		vec_norm = sqrtf(vec_norm);

		if(include_norm)	accel[3] = vec_norm;

		if(IMU_A_DEBUG_MODE > 0)
		{
			printf("A=<%2.4f,", accel[0]);
			printf("%2.4f,", accel[1]);
			printf("%2.4f>", accel[2]);		_delay_ms(5);
			printf(" |%.4f|\r\n", vec_norm);
		}
	}
	
	return success;
}

uint8_t get_9axis_accel_SI(float *accel, uint8_t IMU_num)
{
	uint8_t i;
	uint8_t success;

	success = get_9axis_accel_g(accel, 0, IMU_num);

	for(i = 0; i < 3; i++)
	{
		accel[i] = LITTLE_G*accel[i];
	}

	return success;
}

float get_9axis_accel_resolution(uint8_t IMU_num)
{
	// unless otherwise set elsewhere the resolution is +/- 2.0 g for full scale

	return 2.0;
}


void register_read_print_9axis_accel(uint8_t IMU_num)
{
	uint8_t i;
	uint8_t bytes[0x3F];

	I2C_read_bytes(address_of_IMU_accel(IMU_num), 0, bytes, 0x3F-1);

	for(i = 0x3F+1; i > 0; i--)
	{
		printf("0x%02x: 0x%02x -> ", i-1, bytes[i-1]);
		_delay_ms(5);
		print_uint8_in_binary(bytes[i-1]);
		_delay_ms(2);
	}
}

/* PRIVATE FUNCTIONS - ACCELEROMETER */

uint8_t address_of_IMU_accel(uint8_t IMU_num)
{
	if(IMU_num == 1)
		return BMX055_ACCEL_1_ADDRESS;
	else if(IMU_num == 2)
		return BMX055_ACCEL_2_ADDRESS;
	else
		return 0;	// ERROR
}


/* USER ACCESSIBLE FUNCTIONS - GYROSCOPE */

uint8_t check_IMU_gyro_sensor_exists(uint8_t IMU_num)
{
	uint8_t addr = address_of_IMU_gyro(IMU_num);
	
	I2C_check_device_exists(addr);	// do we need to double check?

	return I2C_check_device_exists(addr);
}

uint8_t verify_IMU_gyro_existance(uint8_t IMU_num)
{
	if((IMU_num == 1)||(IMU_num == 2))
		if(!x9G_existance_confirmed[IMU_num-1])
		{
			printf("BMX055-G-%u not found\r\n", IMU_num);
			_delay_ms(5);
		}
	
	return x9G_existance_confirmed[IMU_num-1];
}

uint8_t get_9axis_gyro_bytes(int16_t *gyro, uint8_t IMU_num)
{
	uint8_t success = 0;	// assume fail until success is detected

	if((success=verify_IMU_gyro_existance(IMU_num)))
	{
		uint8_t bytes[6];
		uint8_t i;
		uint16_t intermediate;

		I2C_read_bytes(address_of_IMU_gyro(IMU_num), ACCD_X_LSB, bytes, 6);
		//I2C_read_bytes(BMX055_GYRO_1_ADDRESS, RATE_X_LSB, bytes, 6);
		
		if(IMU_G_DEBUG_MODE > 2) for(i = 0; i < 6; i++)	printf("%u: %i\r\n", i, bytes[i]);

		// memory map:
		// ADDR		description		 7	6  5  4  3  2  1  0 [bit number]
		// 0x02		x-LSB			 7  6  5  4  3  2  1  0	
		// 0x03		x-MSB			15 14 13 12 11 10  9  8

		for(i = 0; i < 3; i++)
		{
			intermediate = bytes[(2*i)+1];
			intermediate = intermediate<<8;
			intermediate += bytes[2*i];

			gyro[i] = twos_complement_conversion(intermediate, 15);
		}
	}

	if(IMU_G_DEBUG_MODE > 1)
	{
		printf("G%u: [%i, %i, %i]\r\n", IMU_num, gyro[0], gyro[1], gyro[2]);
		_delay_ms(5);
	}
	
	return success;
}

uint8_t get_9axis_gyro_deg(float *gyro, uint8_t IMU_num)
{
	int16_t vals[3];
	uint8_t i;
	uint8_t success;

	success = get_9axis_gyro_bytes(vals, IMU_num);

	// TEST VALUES: minimum and maximum
	//vals[0] = (1<<15);  				// = -1*gyro_resolution
	//vals[1] = 0xFFFF &~ (1<<15);		// =  1*gyro_resolution

	if(success)
	{
		for(i = 0; i < 3; i++)
		{
			gyro[i] = vals[i];				// int16_t -> float
			gyro[i] = gyro[i]/32768.0;		// 32768 = 2^15 = 1<<15
			gyro[i] *= gyro_resolution[IMU_num-1];	
		
		
			if((gyro[i] > 0.9*gyro_resolution[IMU_num-1])||(gyro[i] < -0.9*gyro_resolution[IMU_num-1]))
			{
				printf("WOAH! That was intense on %u\r\n", IMU_num);
				_delay_ms(3000);

				double_9axis_gyro_resolution(IMU_num);
			}
		
		}

		if(IMU_G_DEBUG_MODE > 0)
		{
			printf("G%u=<%2.4f,", IMU_num, gyro[0]);
			printf("%2.4f,", gyro[1]);
			printf("%2.4f>\r\n", gyro[2]);		_delay_ms(5);
		}
	}
	
	return success;
}

uint8_t get_9axis_gyro_SIrad(float *gyro, uint8_t IMU_num)	
{	
	// units: radians/second
	uint8_t i;
	uint8_t success;
	
	success = get_9axis_gyro_deg(gyro, IMU_num);

	for(i = 0; i < 3; i++)
	{
		gyro[i] = gyro[i]*(M_PI/180.0);
	}

	return success;
}

void set_9axis_gyro_resolution(uint8_t bits, uint8_t IMU_num)
{
	uint8_t IMU_ind = IMU_num-1;
	
	if(bits > 0b100) bits = 0b100;

	float new_resolution = 2000.0/((float)(1<<bits));
	
	if(new_resolution != gyro_resolution[IMU_ind])
	{
		I2C_write_byte(address_of_IMU_gyro(IMU_num), 0x0F, bits);

		gyro_resolution[IMU_ind] = new_resolution;
	}
	
	if(IMU_G_DEBUG_MODE > 0)
		printf("gyro res: %.1f ø/s\r\n", gyro_resolution[IMU_ind]);
}

float get_9axis_gyro_resolution(uint8_t IMU_num)
{
	return gyro_resolution[IMU_num-1];

	// note: use "%.1f ø/s" to print the value
}

void double_9axis_gyro_resolution(uint8_t IMU_num)
{
	if(gyro_resolution[IMU_num-1] < 250)
		set_9axis_gyro_resolution(3, IMU_num);	//  250 deg/s

	else if(gyro_resolution[IMU_num-1] < 500)
		set_9axis_gyro_resolution(2, IMU_num);	//  500 deg/s

	else if(gyro_resolution[IMU_num-1] < 1000)
		set_9axis_gyro_resolution(1, IMU_num);	// 1000 deg/s

	else if(gyro_resolution[IMU_num-1] < 2000)
		set_9axis_gyro_resolution(0, IMU_num);	// 2000 deg/s
}

void register_read_print_9axis_gyro(uint8_t IMU_num)
{
	uint8_t i;
	uint8_t bytes[0x3F];

	I2C_read_bytes(address_of_IMU_gyro(IMU_num), 0, bytes, 0x3F-1);

	for(i = 0x3F+1; i > 0; i--)
	{
		printf("0x%02x: 0x%02x -> ", i-1, bytes[i-1]);
		_delay_ms(5);
		print_uint8_in_binary(bytes[i-1]);
		_delay_ms(2);
	}
}

uint8_t address_of_IMU_gyro(uint8_t IMU_num)
{
	if(IMU_num == 1)
		return BMX055_GYRO_1_ADDRESS;
	else if(IMU_num == 2)
		return BMX055_GYRO_2_ADDRESS;

	return 0;	// ERROR
}

/* USER ACCESSIBLE FUNCTIONS - MAGNETOMETER */

uint8_t check_IMU_mag_sensor_exists(uint8_t IMU_num)
{
	uint8_t addr = address_of_IMU_mag(IMU_num);
	
	I2C_check_device_exists(addr);	// do we need to double check?

	return I2C_check_device_exists(addr);
}

uint8_t verify_IMU_mag_existance(uint8_t IMU_num)
{
	if((IMU_num == 1)||(IMU_num == 2))
		if(!x9M_existance_confirmed[IMU_num-1])
		{
			printf("BMX055-M-%u not found\r\n", IMU_num);
			_delay_ms(5);
		}
	
	return x9M_existance_confirmed[IMU_num-1];
}

uint8_t get_9axis_mag_bytes(int16_t* mag, uint8_t IMU_num)
{
	uint8_t success = 0;	// assume fail until success is detected
	uint8_t IMU_addr = address_of_IMU_mag(IMU_num);

	if((success=verify_IMU_mag_existance(IMU_num)))
	{
		uint8_t bytes[6];
		uint8_t i;
		uint16_t intermediate;

		I2C_read_bytes(IMU_addr, M_DATA_X_LSB, bytes, 6);
	
		if(IMU_M_DEBUG_MODE > 2) for(i = 0; i < 6; i++)	printf("%u: %i\r\n", i, bytes[i]);

		//load_mag_fake_bytes(bytes);

		// The magnetometer is the only component on the BMX055 with different resolution on X,Y,(Z) axes
		// memory map:
		// ADDR		description		 7	6  5  4  3  2  1  0		[bit number]
		// 0x42		x-LSB			 4  3  2  1  0  x  x st		st = self-test
		// 0x43		x-MSB			12 11 10  9  8  7  6  5
		
		// 0x44		y-LSB			 4  3  2  1  0  x  x st
		// 0x45		y-MSB			12 11 10  9  8  7  6  5
		
		// 0x46		z-LSB			 6  5  4  3  2  1  0 st
		// 0x47		z-MSB			14 13 12 11 10  9  8  7

		for(i = 0; i < 2; i++)	// handles X and Y axes only!
		{
			intermediate = bytes[(2*i)+1];
			intermediate = intermediate<<5;
			intermediate += (bytes[2*i]>>3)&0x1F;

			// check self test flag???
			if(!(bytes[2*i]&0x01))
			{
				printf("st");
				success = 0;
			}

			mag[i] = twos_complement_conversion(intermediate, 12);
		}

		// Z axis
		i = 2;
		intermediate = bytes[(2*i)+1];
		intermediate = intermediate<<7;
		intermediate += (bytes[2*i]>>1)&0x7F;

		// check self test flag???
		if(!(bytes[2*i]&0x01))
		{
			printf("st");
			success = 0;
		}

		mag[i] = twos_complement_conversion(intermediate, 14);

	}

	if(IMU_M_DEBUG_MODE > 1)
	{
		printf("M%u: [%i, %i, %i]\r\n", IMU_num, mag[0], mag[1], mag[2]);
		_delay_ms(5);
	}

	return success;
}

uint8_t get_9axis_mag_uT(float *mag, uint8_t IMU_num)
{
	int16_t mag_bytes[3];
	uint8_t success;
	float vec_norm;

	// measurable range (if all bits are used) conversion range		|	datasheet
	//	axis	bits	negmax		posmax		min		max				min		max
	//	X		13		-4096		4095	-1228.8	uT	1228.5 uT	  -1300 uT	1300 uT
	//	Y		13		-4096		4095	-1228.8	uT	1228.5 uT	  -1300 uT	1300 uT
	//	Z		15		-16384		16383	-4915.2 uT	4914.9 uT	  -2500 uT	2500 uT

	success = get_9axis_mag_bytes(mag_bytes, IMU_num);

	for(uint8_t i = 0; i < 3; i++)
	{
		mag[i] = (float)mag_bytes[i]*0.3;	// uT
		vec_norm += mag[i]*mag[i];	// sum, for L2 norm of magnetic field vectors
	}

	vec_norm = sqrtf(vec_norm);
	

	if(IMU_M_DEBUG_MODE > 0)
	{
		printf("M%u=<%.1f,", IMU_num, mag[0]);
		printf("%.1f,", mag[1]);	_delay_ms(5);
		printf("%.1f>", mag[2]);	_delay_ms(5);
		printf(" |%.1f|\r\n", vec_norm);
	}

	return success;
}

float get_9axis_mag_resolution(uint8_t IMU_num)
{
	// unless set elsewhere (check the datasheet for implementation)
	// the full scale of the magnetometer is +/- 1228.5 for x,y and +/- 4914.9 for z
	// the resolution is 0.3 uT per bit
	// see the conversion code for details

	return 1228.5;
}


// debugging, read and print contents of every single magnetometer register
void register_read_print_9axis_mag(uint8_t IMU_num)
{
	uint8_t IMU_addr = address_of_IMU_mag(IMU_num);

	uint8_t i;
	uint8_t start_byte = 0x40;
	uint8_t end_byte = 0x52;
	uint8_t num_bytes = end_byte - start_byte + 1;
	uint8_t bytes[num_bytes];

	I2C_read_bytes(IMU_addr, start_byte, bytes, num_bytes);

	i = num_bytes;
	for(uint8_t reg = end_byte; reg >= start_byte; reg--)
	{
		i--;
		printf("0x%02x: 0x%02x -> ", reg, bytes[i]);
		_delay_ms(5);
		print_uint8_in_binary(bytes[i]);
		_delay_ms(2);
	}
}

void single_register_read_print_9axis_mag(uint8_t reg, uint8_t IMU_num)
{
	uint8_t byte;
	uint8_t IMU_addr = address_of_IMU_mag(IMU_num);

	I2C_read_bytes(IMU_addr, reg, byte, 1);

	printf("0x%02x: 0x%02x -> ", reg, byte);
	_delay_ms(5);
	print_uint8_in_binary(byte);
	_delay_ms(2);
	
}

uint8_t address_of_IMU_mag(uint8_t IMU_num)
{
	if(IMU_num == 1)
		return BMX055_MAG_1_ADDRESS;
	else if(IMU_num == 2)
		return BMX055_MAG_2_ADDRESS;

	return 0;	// ERROR
}

void load_mag_fake_bytes(uint8_t* mag_bytes)
{
	// The magnetometer is the only component on the BMX055 with different resolution on X,Y,(Z) axes
	// memory map:
	// ADDR		description		 7	6  5  4  3  2  1  0		[bit number]
	// 0x42		x-LSB			 4  3  2  1  0  x  x st		st = self-test
	// 0x43		x-MSB			12 11 10  9  8  7  6  5
	
	// 0x44		y-LSB			 4  3  2  1  0  x  x st
	// 0x45		y-MSB			12 11 10  9  8  7  6  5
	
	// 0x46		z-LSB			 6  5  4  3  2  1  0 st
	// 0x47		z-MSB			14 13 12 11 10  9  8  7

	uint8_t preval;
	uint8_t i;

	printf("fake magnet data\r\n");
	_delay_ms(5);

	// x-data
	i = 0;	// x-LSB
	preval = mag_bytes[i];
	//mag_bytes[i] = 0b00000000;
	mag_bytes[i] = 0b11111000; 
	printf("%u -> %u\r\n", preval, mag_bytes[i]);
	i = 1;	// x-MSB
	preval = mag_bytes[i];
	//mag_bytes[i] = 0b10000000;
	mag_bytes[i] = 0b01111111;
	printf("%u -> %u\r\n", preval, mag_bytes[i]);

	// y-data
	i = 2;	// y-LSB
	preval = mag_bytes[i];
	//mag_bytes[i] = 0b00000000;
	mag_bytes[i] = 0b11111000;
	printf("%u -> %u\r\n", preval, mag_bytes[i]);
	i = 3;	// y-MSB
	preval = mag_bytes[i];
	//mag_bytes[i] = 0b10000000;
	mag_bytes[i] = 0b01111111;
	printf("%u -> %u\r\n", preval, mag_bytes[i]);
	
	// z-data
	i = 4;	// z-LSB
	preval = mag_bytes[i];
	//mag_bytes[i] = 0b00000000;
	mag_bytes[i] = 0b11111110;
	printf("%u -> %u\r\n", preval, mag_bytes[i]);
	i = 5;	// z-MSB
	preval = mag_bytes[i];
	//mag_bytes[i] = 0b10000000;
	mag_bytes[i] = 0b01111111;
	printf("%u -> %u\r\n", preval, mag_bytes[i]);
}


/* INITIALIZATION */

uint8_t init_9axis_accel(void)
{
	if(check_IMU_accel_sensor_exists(1) == 1)	x9A_existance_confirmed[0] = 1;
	else										x9A_existance_confirmed[0] = 0;

	if(check_IMU_accel_sensor_exists(2) == 1)	x9A_existance_confirmed[1] = 1;
	else										x9A_existance_confirmed[1] = 0;

	return (x9A_existance_confirmed[0] & x9A_existance_confirmed[1]);
}

uint8_t init_9axis_gyro(void)
{
	gyro_resolution[0] = DEFAULT_GYRO_RESOLUTION;
	gyro_resolution[1] = DEFAULT_GYRO_RESOLUTION;
	
	// values were clipping in normal operation on 125 deg/s and in fast operation on 250 deg/s

	if(check_IMU_gyro_sensor_exists(1) == 1)
	{
		x9G_existance_confirmed[0] = 1;

		//set_9axis_gyro_resolution(0, 1);	// 2000 deg/s
		//set_9axis_gyro_resolution(1, 1);	// 1000 deg/s
		set_9axis_gyro_resolution(2, 1);	//  500 deg/s
		//set_9axis_gyro_resolution(3, 1);	//  250 deg/s
		//set_9axis_gyro_resolution(4, 1);	//  125 deg/s
	}
	else
		x9G_existance_confirmed[0] = 0;

	if(check_IMU_gyro_sensor_exists(2) == 1)
	{
		x9G_existance_confirmed[1] = 1;

		//set_9axis_gyro_resolution(0, 2);	// 2000 deg/s
		//set_9axis_gyro_resolution(1, 2);	// 1000 deg/s
		set_9axis_gyro_resolution(2, 2);	//  500 deg/s
		//set_9axis_gyro_resolution(3, 2);	//  250 deg/s
		//set_9axis_gyro_resolution(4, 2);	//  125 deg/s
	}
	else
		x9G_existance_confirmed[1] = 0;

	// initialize the mean/standard-deviation module, for self-calibration
	//init_M_SD_module(100);	// (TODO) (under test development)

	return (x9G_existance_confirmed[0] & x9G_existance_confirmed[1]);
}

uint8_t init_9axis_mag(void)
{
	if(check_IMU_mag_sensor_exists(1) == 1)
	{
		x9M_existance_confirmed[0] = 1;
		
		I2C_write_byte(BMX055_MAG_1_ADDRESS, MAG_PWR_CTRL, 0x01);	// power on
		_delay_ms(20);
		I2C_write_byte(BMX055_MAG_1_ADDRESS, MAG_MODE_CTRL, 0x00);	// normal mode
		_delay_ms(10);
	}
	else
		x9M_existance_confirmed[0] = 0;

	if(check_IMU_mag_sensor_exists(2) == 1)
	{
		x9M_existance_confirmed[1] = 1;
		
		I2C_write_byte(BMX055_MAG_2_ADDRESS, MAG_PWR_CTRL, 0x01);	// power on
		_delay_ms(20);
		I2C_write_byte(BMX055_MAG_2_ADDRESS, MAG_MODE_CTRL, 0x00);	// normal mode
		_delay_ms(10);
	}
	else
		x9M_existance_confirmed[1] = 0;

	return (x9M_existance_confirmed[0] & x9M_existance_confirmed[1]);
}

