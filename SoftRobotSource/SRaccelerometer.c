// SRaccelerometer.c

#include "SRaccelerometer.h"

#include <util/delay.h>

// Internal Register Addresses
#define XDATA 0x08
#define YDATA 0x09
#define ZDATA 0x0A


uint8_t accelerometer_existance_confirmed = 0;	// boolean, set in init

/* USER ACCESSIBLE FUNCTIONS */

uint8_t check_accel_exists(void)
{
	return 0;	// TODO: on-board accelerometer has not yet been implemented!
}

// prints the measurements of the accelerometer to stdout
void accel_status(void)
{
	uint8_t x_data, y_data, z_data;
	float x_value, y_value, z_value;

	printf("accelerometer\r\n");
	
	if(accelerometer_existance_confirmed)
	{
		//gather raw data
		x_data = accel_read(XDATA);
		y_data = accel_read(YDATA);
		z_data = accel_read(ZDATA);
		

		x_value = accel_data_to_number(x_data);
		y_value = accel_data_to_number(y_data);
		z_value = accel_data_to_number(z_data);

		

		printf("X RAW: %u\r\n", x_data);
		printf("Y RAW: %u\r\n", y_data);
		printf("Z RAW: %u\r\n", z_data);
		
		printf("X-AXIS: %f\r\n", x_value);
		_delay_ms(1);
		printf("Y-AXIS: %f\r\n", y_value);
		_delay_ms(1);
		printf("Z-AXIS: %f\r\n", z_value);
		_delay_ms(1);
		
	}
	
	else
		printf("DOES NOT EXIST\r\n");
}


/* INTERNAL FUNCTIONS */

uint8_t accel_read(uint8_t read_address)
{
	return 0;	
}



void accel_write(uint8_t addr, uint8_t data)
{

}

// convert raw data into decimal with sign and units of g's
float accel_data_to_number(uint8_t axis_data)
{
	//uint8_t accel_value;
	float answer = 0;

	


	return answer;
}




/* INITIALIZATION */

uint8_t init_accel(void)
{	
	if(check_accel_exists() == 1)
	{
		accelerometer_existance_confirmed = 1;	
	}

	else
		accelerometer_existance_confirmed = 0;	// redundant

	return accelerometer_existance_confirmed;
}


// detect a shake
void shake(uint8_t tilt)
{						
	
}

// detect a tap
void tap(uint8_t tilt)
{							
	
}

//interrupt function for PCB mounted accelerometer
ISR(PORTB_INT0_vect)
{		
	// Accelerometer (ADXL362) has 2 interrupt lines connected to the Xmega:
	// ACCEL_INT1 on portBpin6
	// ACCEL_INT2 on portBpin7
	
	// it is currently unknown if this interrupt is working properly
	printf("A-interrupt!!\r\n");

	return 0;
}