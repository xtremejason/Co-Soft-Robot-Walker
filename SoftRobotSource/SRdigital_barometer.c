//SRdigital_barometer.c

#include "SRdigital_barometer.h"

#include <util/delay.h> // A small delay is necessary between request and read
//#include <inttypes.h>

#include "software_clock.h"
#include "SRmisc_functions.h"
#include "SoftRobotI2C.h"

// we are using Measurement Specialties MS5637-02BA03 Low Voltage Barometric Pressure Sensor
#define BPS_I2C_ADDRESS			0x76	// this is a standard 7-BIT I2C address!

// DEBUG STATEMENTS
#define BPS_DEBUG_PRINT 		0		// no printing
//#define BPS_DEBUG_PRINT 		1		// print errors
//#define BPS_DEBUG_PRINT 		2		// print conversion intermediate results
//#define BPS_DEBUG_PRINT 		3		// print byte-by-byte I2C response

#define BPS_TEST_MODE			0		// real data mode
//#define BPS_TEST_MODE			1		// test data mode


#define BPS_RESET				0x1E
#define BPS_PROM_READ_ADDR		0xA0

#define BPS_CONVERT_D1_LOW_RES	0x40
#define BPS_CONVERT_D2_LOW_RES	0x50

#define D1_TEST_VAL		6465444
#define D2_TEST_VAL		8077636

// operating parameters
#define DBM_CONVERSION_TIME_ms	20		// this depends on resolution, see table in comments

/* There are different resolution modes, referred to as OSR, which affect the function of the MS5637 as follows

OSR		I(uA)	Tc(ms)	Pr(mbar)	Tr(C)
8192	20.09	16.44	0.016		0.002		*HI-RES*
4096	10.05	 8.22	0.021		0.003
2048	 5.02	 4.13	0.028		0.004
1024	 2.51	 2.08	0.039		0.006
 512	 1.26	 1.06	0.062		0.009
 256	 0.63	 0.54	0.110		0.012		*LOW-RES*

Tr = Temperature Resolution RMS
Pr = Pressure Resolution RMS
Tc = Conversion Time
I = Current Consumed @ 1 sample/sec (@ Vdd = 3.0 V)		*/


// GLOBAL VARIABLES:
uint8_t BPS_existance_confirmed = 0;	// boolean, set in init
uint16_t BPS_last_time_ms = 0;			// for keeping data fresh

// data variables, these get updated every new read request
uint32_t D1;	// needed for pressure
uint32_t D2;	// needed for temperature and pressure

// calibration coefficients, loaded during initialization
uint16_t C_coef[6];
/* Typical Values and Definitions:
C1 43704	pressure sensitivity
C2 44331	pressure offset
C3 25488	temperature coefficient of pressure sensitivity
C4 25835	temperature coefficient of pressure offset
C5 28634	reference temperature
C6 26274	temperature coefficient of the temperature	*/

// last conversion results, currently unused
float BPS_last_temp_C = 0;
float BPS_last_pressure_mbar = 0;


/* USER ACCESSIBLE FUNCTIONS */

uint8_t check_BPS_exists(void)
{
	//check_all_of_I2C();
	//_delay_ms(20);

	I2C_check_device_exists(BPS_I2C_ADDRESS);

	return I2C_check_device_exists(BPS_I2C_ADDRESS);
}

float get_BPS_pressure_mbar(void)
{
	int32_t dT;
	int64_t OFF;	//int64_t OFF_correct = 5764707214L;
	int64_t SENS;	//int64_t SENS_correct = 3039050829L;
	int64_t P;		//int64_t P_correct = 110002L;
	float P_meas;
	
	// load working-size calibration constants
	int64_t C1L = (int32_t)C_coef[0];
	int64_t C2L = (int32_t)C_coef[1];
	int32_t C3L = (int32_t)C_coef[2];
	int32_t C4L = (int32_t)C_coef[3];
	int32_t C5L = (int32_t)C_coef[4];


	if(BPS_existance_confirmed)
	{
		// load new input values: D1, D2 (if applicable/stale)
		BPS_read_convert_data();
	
		// compute dT (difference between actual and reference temperature):
		dT = D2 - (C5L<<8);					// = D2 - C5L*256, 2^8 = 256
		if(BPS_DEBUG_PRINT >= 2) {printf("dT %li\r\n", dT); _delay_ms(2);}

		// compute OFF (offset at actual temperature):
		OFF = (C2L<<17) + ((C4L*dT)>>6);	// C2L*131072 + (C4L*dT)/64, 2^17 = 131072, 2^6 = 64

		// compute SENS (sensitivity at actual temperature):
		SENS = (C1L<<16) + ((C3L*dT)>>7);	// = C1L*65536 + (C3*dT)/128, 2^16 = 65536, 2^7 = 128

		// compute P (temperature compensated pressure):
		P = ( ((D1*SENS)>>21) - OFF)>>15;	// = (D1*SENS/2097152 - OFF)/32768, 2^21 = 2097152, 2^15 = 32768
		if(BPS_DEBUG_PRINT >= 2) {printf("P %li\r\n", P); _delay_ms(2);}

		P_meas = P*0.01;	// example: P = 110002 -> P_meas = 1100.02 mbar

		BPS_last_pressure_mbar = P_meas;
	}
	else	P_meas = 0;

	return P_meas;
}

float get_BPS_temperatue_C(void)
{
	int32_t dT;
	int32_t TEMP;
	
	int32_t C5L = (int32_t)C_coef[4];
	int32_t C6L = (int32_t)C_coef[5];

	float temp_C;

	if(BPS_existance_confirmed)
	{
		// load new input values: D1, D2 (if applicable/stale)
		BPS_read_convert_data();
		
		// compute dT (difference between actual and reference temperature):
		dT = D2 - (C5L<<8);				// = D2 - C5L*256, 2^8 = 256
		if(BPS_DEBUG_PRINT >= 2) {printf("dT %li\r\n", dT); _delay_ms(2);}

		// compute TEMP (actual temperature):
		TEMP = 2000 + ((dT*C6L)>>23);	// = 2000 + dT*(C6/8388608), 2^23 = 8388608
		if(BPS_DEBUG_PRINT >= 2) {printf("TEMP %li\r\n", TEMP); _delay_ms(2);}

		temp_C = (float)TEMP/100.0;

		BPS_last_temp_C = temp_C;
	}
	else	temp_C = 0;

	return temp_C;
}

uint8_t BPS_data_is_stale(void)
{
	uint16_t time_ms = get_milliseconds();
	uint16_t t_diff = compute_tdiff_ms(BPS_last_time_ms, time_ms);
	
	if(BPS_DEBUG_PRINT > 1)	printf("Tdelta %u\r\n", t_diff);

	if(t_diff > BPS_DATA_EXPIRATION_TIME_ms)
		return 1;
	
	return 0;
}


/* INTERNAL FUNCTIONS - HIGH LEVEL */

void BPS_read_convert_data(void)
{
	if(BPS_data_is_stale())
	{
		if(BPS_TEST_MODE)
		{
			BPS_read_convert_D1_TEST();
			BPS_read_convert_D2_TEST();
		}
		else
		{
			BPS_read_convert_D1();
			BPS_read_convert_D2();
		}

		if(BPS_DEBUG_PRINT >= 2)
		{
			printf("D1: %li\r\n", D1);
			printf("D2: %li\r\n", D2);
		}

		BPS_last_time_ms = get_milliseconds();
	}
	else
	{
		if(BPS_DEBUG_PRINT) printf("no update\r\n");
	}
}

void BPS_read_convert_D1(void)
{
	uint8_t read_bytes[3];

	I2C_write_command(BPS_I2C_ADDRESS, BPS_CONVERT_D1_LOW_RES);

	// delay for conversion, device freezes if no delay is given
	_delay_ms(DBM_CONVERSION_TIME_ms);

	I2C_read_bytes(BPS_I2C_ADDRESS, 0, read_bytes, 3);
	
	if(BPS_DEBUG_PRINT >= 3)
	{
		printf("D1: ");
		for(uint8_t i = 0; i < 3; i++)
		{
			printf("%2x ", read_bytes[i]);
		}	printf("\r\n");
	}
	
	D1 = build_uint32_from_byte_array(read_bytes, 3);
}

void BPS_read_convert_D2(void)
{
	uint8_t read_bytes[3];

	I2C_write_command(BPS_I2C_ADDRESS, BPS_CONVERT_D2_LOW_RES);

	// delay for conversion, device freezes if no delay is given
	_delay_ms(DBM_CONVERSION_TIME_ms);

	I2C_read_bytes(BPS_I2C_ADDRESS, 0, read_bytes, 3);
	
	if(BPS_DEBUG_PRINT >= 3)
	{
		printf("D2: ");
		for(uint8_t i = 0; i < 3; i++)
		{
			printf("%2x ", read_bytes[i]);
		}	printf("\r\n");
	}
	
	D2 = build_uint32_from_byte_array(read_bytes, 3);
}

void BPS_read_calibration_PROM(void)
{
	uint8_t i;
	uint8_t read_bytes[2];
	uint8_t addr;

	// note: BPS_PROM_READ_ADDR contains CRC info, is not used for calibration

	for(i = 1; i < 7; i++)
	{
		addr = BPS_PROM_READ_ADDR + 2*i;
		I2C_read_bytes(BPS_I2C_ADDRESS, addr, read_bytes, 2);
		C_coef[i-1] = (read_bytes[0]<<8) + read_bytes[1];
	}
}


/* INTERNAL FUNCTIONS - LOW LEVEL */

void BPS_reset_sequence(void)
{
	if(BPS_DEBUG_PRINT >= 1)	printf("BPS reset\r\n");
	
	I2C_write_command(BPS_I2C_ADDRESS, BPS_RESET);

	_delay_ms(10);	// unknown if any delay is necessary

	if(check_BPS_exists() == 1)
	{
		BPS_existance_confirmed = 1;
		if(BPS_DEBUG_PRINT > 1)	printf("found DBM\r\n");
	}
	else
	{
		BPS_existance_confirmed = 0;
		if(BPS_DEBUG_PRINT > 0)	printf("no DBM\r\n");
	}
}


/* DEBUG FUNCTIONS */

void BPS_read_convert_D1_TEST(void)
{
	uint8_t read_bytes[3];

	printf("D1 TEST-\r\n");

	// delay for conversion, device freezes if no delay is given
	_delay_ms(DBM_CONVERSION_TIME_ms);

	read_bytes[0] = (D1_TEST_VAL>>16)&0xFF;
	read_bytes[1] = (D1_TEST_VAL>>8)&0xFF;
	read_bytes[2] = (D1_TEST_VAL>>0)&0xFF;
	
	if(BPS_DEBUG_PRINT >= 3)
	{
		printf("D1: ");
		for(uint8_t i = 0; i < 3; i++)
		{
			printf("%2x ", read_bytes[i]);
		}	printf("\r\n");
	}
	
	D1 = build_uint32_from_byte_array(read_bytes, 3);
}


void BPS_read_convert_D2_TEST(void)
{
	uint8_t read_bytes[3];

	printf("D2 TEST-\r\n");

	// delay for conversion, device freezes if no delay is given
	_delay_ms(DBM_CONVERSION_TIME_ms);

	read_bytes[0] = (D2_TEST_VAL>>16)&0xFF;
	read_bytes[1] = (D2_TEST_VAL>>8)&0xFF;
	read_bytes[2] = (D2_TEST_VAL>>0)&0xFF;
	
	if(BPS_DEBUG_PRINT >= 3)
	{
		printf("D2: ");
		for(uint8_t i = 0; i < 3; i++)
		{
			printf("%2x ", read_bytes[i]);
		}	printf("\r\n");
	}
	
	D2 = build_uint32_from_byte_array(read_bytes, 3);
}


void BPS_read_calibration_PROM_TEST(void)
{
	C_coef[0] = 46372;	// C1
	C_coef[1] = 43981;	// C2
	C_coef[2] = 29059;	// C3
	C_coef[3] = 27842;	// C4
	C_coef[4] = 31553;	// C5
	C_coef[5] = 28165;	// C6
}

void BPS_print_calibration_PROM(void)
{
	for(uint8_t i = 0; i < 6; i++)
	{
		printf("C%u %u\r\n", i+1, C_coef[i]); _delay_ms(5);
	}
}


/* INITIALIZATION */

uint8_t init_BPS(void)
{
	if(check_BPS_exists() == 1)
	{
		BPS_existance_confirmed = 1;		
		if(BPS_DEBUG_PRINT > 1)		printf("found DBM\r\n");
	}
	
	else
	{
		BPS_existance_confirmed = 0;
		if(BPS_DEBUG_PRINT > 0)		printf("no DBM\r\n");
	}

	if(BPS_existance_confirmed)
	{
		// load factory calibrated data (C1 .. C6)
		if(BPS_TEST_MODE)	BPS_read_calibration_PROM_TEST();
		else				BPS_read_calibration_PROM();
	
		BPS_print_calibration_PROM();
	}

	return BPS_existance_confirmed;
}