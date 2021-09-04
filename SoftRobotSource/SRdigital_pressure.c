//SRdigital_pressure.c

#include "SRdigital_pressure.h"

#include "SoftRobotI2C.h"

// DEBUG STATEMENTS
#define DPS_DEBUG_PRINT 0		// 0,1,2 are the levels of debug

uint16_t DPS_last_time_ms = 0; // for timekeeping, move to software_clock soon (TODO)


uint8_t DPS_existance_confirmed = 0;	// boolean, set in init

float DPS_last_temp = 0;
float DPS_last_pressure = 0;

uint16_t readcount = 0;


/* USER ACCESSIBLE FUNCTIONS */

uint8_t check_p_sensor_exists()
{
	return I2C_check_device_exists(DPS_ADDRESS);
}

// read and print data from the pressure sensor
float get_digital_pressure(void)
{
	uint8_t status_bits;		// 2 MSBs only
	uint16_t pressure_bits;		// all except 2 MSBs
	uint16_t temperature_bits;	// all except 5 LSBs

	float p_meas = 0;
	float t_meas;
	
	//toggle_LED(1);	// DEBUG
	
	if(digital_pressure_data_is_stale())
	{
		if(!DPS_existance_confirmed)
			printf("DPS not found\r\n");
	
		else
		{
			DPS_get_bits(&status_bits, &pressure_bits, &temperature_bits);

			p_meas = DPS_pressure_conversion(pressure_bits);

			t_meas = DPS_temperature_conversion(temperature_bits);

			DPS_last_temp = t_meas;		// GLOBAL ASSIGNMENT		
		
			// record the time that the measurement occurred:
			DPS_last_time_ms = get_milliseconds();

			DPS_last_pressure = p_meas;
		}
		
		return p_meas;
	}
	else
		return get_digital_pressure_lastread();
}

// read and print data from the pressure sensor
float get_air_temp(void)
{
	return DPS_last_temp;
}

float get_digital_pressure_lastread(void)
{
	if(DPS_DEBUG_PRINT > 0)
		printf("X");
	
	return DPS_last_pressure;
}


uint8_t digital_pressure_data_is_stale(void)
{
	uint16_t time_ms = get_milliseconds();
		
	uint16_t t_diff = compute_tdiff_ms(DPS_last_time_ms, time_ms);
	
	if(t_diff > DPS_DATA_EXPIRATION_TIME_ms)
		return 1;
		
	return 0;	
}

float DPS_pressure_conversion(uint16_t pressure_bits)
{
	// pressure_bits: all significant except 2 MSBs
	
	float pressure_value;

	// we have 'A' type with Pmax = 15 PSI, Pmin = -15 PSI
	// pressure equation (pg 4 datasheet):
	// [pressure bits] = ((0.8 * 16383)/(30[psi]))*(Papplied + 15[psi]) + 0.1*16383

	// note: 0x2000 = 8192 in the pressure bits equals 0 PSI difference
	//
	// +/- 1 bit = +/- 0.0022895 PSI
	// using equation as given in datasheet yields 0 PSI difference -> 0.001144 PSI
	// therefore this correction has been applied to the equation

	//p_meas = (((float)pressure_bits - 1638.3)/436.88) - 15.0;
	pressure_value = (((float)pressure_bits - 1638.3)/436.88) - 15.001144;	// corrected

	return pressure_value;
}

float DPS_temperature_conversion(uint16_t temperature_bits)
{
	// temperature_bits: all significant except 5 LSBs
	
	float temperature_value;

	//printf("tbits: %04x\r\n", temperature_bits);
			
	// temp equation (pg 4 of datasheet, under the linear plot):
	// 2047 counts per 200 degC range
	// 0 counts = -50 degC
	// 2047 counts = 150 degC
	temperature_value = (float)temperature_bits*(200.0/2047.0) - 50.0; // in Centigrade/Celsius

	return temperature_value;	
}

uint8_t DPS_status_conversion(uint8_t status_bits)
{
	// status_bits: only 2 MSBs are significant
	
	// note: status_bits
	// 0 -> normal		(good data)
	// 1 -> command		(reserved)
	// 2 -> busy		(data returned is from the last measurement / 'stale data')
	// 3 -> diagnostic	(fault detected)
	//
	// note: if ever status 3 appears, a power-on reset (power off-on) must occur to reset the status
	//

	return status_bits;
}

void DPS_get_bits(uint8_t *status_bits, uint16_t *pressure_bits, uint16_t *temperature_bits)
{
	uint8_t sep_bytes[4];

	uint8_t s_bits;
	uint16_t p_bits;
	uint16_t t_bits;

	I2C_read_bytes(DPS_ADDRESS, 0, sep_bytes, 4);	// TODO: unknown compiler warn here of implicit declaration

	/*
	// simulated read (DEBUGGING)
	sep_bytes[0] = 0x34;
	sep_bytes[1] = 0x34;
	sep_bytes[2] = 0x34;
	sep_bytes[3] = 0x34;
	_delay_ms(1);
	/*/

	//printf("Ebits: %02x %02x %02x %02x\r\n", sep_bytes[0], sep_bytes[1], sep_bytes[2], sep_bytes[3]);
	
	s_bits = (sep_bytes[0] >> 6) & 0b00000011;
			
	if(DPS_status_conversion(s_bits) > 0)
		printf("ERROR: DPS status %i\r\n", s_bits);

	p_bits = sep_bytes[0] & 0b00111111;
	p_bits = p_bits << 8;
	p_bits = p_bits | sep_bytes[1];
		
	t_bits = sep_bytes[2] << 3;
	t_bits = t_bits | ((sep_bytes[3] >> 5) & 0b00000111);

	*status_bits = s_bits;
	*pressure_bits = p_bits;
	*temperature_bits = t_bits;
}




/* INITIALIZATION */

// initialize digital pressure controller
uint8_t init_DPS(void)
{

	if(check_p_sensor_exists() == 1)
	{
		DPS_existance_confirmed = 1;
		if(DPS_DEBUG_PRINT > 0)
			printf("found DPS\r\n");
	}
	else
	{
		DPS_existance_confirmed = 0;
		if(DPS_DEBUG_PRINT > 0)
			printf("missing\r\n");
	}

	return DPS_existance_confirmed;
}

