// SRmisc_functions.c

#include "SRmisc_functions.h"

#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>			// for printf

#define kPa2PSI		0.14503774	// 1 kPa = 0.14503774 PSI (approx)


// Mean and Standard Deviation Module variables
uint16_t M_SD_target_num_samples;
uint16_t M_SD_num_samples;
float M_SD_mean;
float M_SD_variance;


// ----- bitwise operations:

uint8_t flag_is_on(uint8_t flags, uint8_t flag_num)
{
	if(flags & (1 << (flag_num-1)))
	return 1;
	
	return 0;
}

uint8_t make_flag(uint8_t prior_flags, uint8_t flag_num)
{
	uint8_t new_flags;

	new_flags = prior_flags | (1 << (flag_num-1));

	return new_flags;
}

// ----- bytewise operations:

uint32_t build_uint32_from_byte_array(uint8_t bytes[], uint8_t num_bytes)
{
	uint32_t big_int;
	uint8_t i;

	big_int = 0;

	for(i = 0; i < num_bytes; i++)
	{
		big_int = big_int<<8;
		big_int += bytes[i];
	}
	
	return big_int;
}

int16_t twos_complement_conversion(uint16_t unsigned_value, uint8_t sign_bit_pos)
{
	uint16_t intermediate;
	int16_t result;
	uint8_t sign;

	// extract the sign bit from the bit sequence (the MSB)
	sign = (unsigned_value>>sign_bit_pos)&0x0001;

	if(sign != 0)	// a nonzero sign bit indicates a negative value
	{
		intermediate = unsigned_value &~ (1<<sign_bit_pos);		// clear sign bit
		result = (1<<(sign_bit_pos)) - intermediate;
		result = result * (-1);
	}

	else
	{
		result = unsigned_value;
	}

	return result;
}


// ----- unit conversions:

float kPa_to_PSI(float kPa)
{
	// 1 kPa = 0.14503774 PSI (approx)
	
	return kPa2PSI*kPa;
}

float mbar_to_PSI(float mbar)
{
	// 1 mbar = 0.1 kPa

	return 0.1*kPa2PSI*mbar;
}

float celsius_to_farenheit(float celsius)
{
	return celsius * (9.0/5.0) + 32.0; // exact
}

float farenheit_to_celsius(float farenheit)
{
	return (farenheit - 32.0) * (5.0/9.0); // exact
}

// ----- math operations:

void OLS_solver(uint8_t x_points[], float y_points[], uint8_t num_points, float *slope_soln, float *yint_soln)
{
	// using Ordinary Least Squares to fit the line to these 2D points, you get
	// slope = sum[(x_i - x_mean)*(y_i - y_mean)]/sum[(x_i - x_mean)^2]
	// y_int = y_mean - slope*x_mean

	float x_sum, y_sum, x_mean, y_mean;
	float numerator, denominator;

	x_sum = 0;
	y_sum = 0;

	for(uint8_t i = 0; i < num_points; i++)
	{
		x_sum += x_points[i];
		y_sum += y_points[i];
	}

	for(uint8_t i = 0; i < num_points; i++)
	{
		numerator += (x_points[i] - x_mean)*(y_points[i] - y_mean);
		denominator += (x_points[i] - x_mean)*(x_points[i] - x_mean);
	}

	x_mean = x_sum/num_points;
	y_mean = y_sum/num_points;

	numerator = 0;			//sum[(x_i - x_mean)*(y_i - y_mean)]
	denominator = 0;		//sum[(x_i - x_mean)^2]

	for(uint8_t i = 0; i < num_points; i++)
	{
		numerator += (x_points[i] - x_mean)*(y_points[i] - y_mean);
		denominator += (x_points[i] - x_mean)*(x_points[i] - x_mean);
	}

	*slope_soln = numerator/denominator;
	*yint_soln = y_mean - (*slope_soln)*x_mean;
}

uint8_t find_median_of_3(uint8_t* meas)
{
	// Finds the median of 3 numbers by finding the max, finding the min, and returning the other value
	uint8_t mini = 0;
	uint8_t maxi = 0;
	uint8_t medi = 0;
	uint8_t min = 0;
	uint8_t max = 0xFF;

	for (uint8_t i = 0; i < 3; i++)
	{
		if(meas[i] < max)
		{
			max = meas[i];
			maxi = i;
		}
		if(meas[i] > min)
		{
			min = meas[i];
			mini = i;
		}
	}
	for (medi = 0; medi < 3; medi++)
	{
		if((medi != maxi) & (medi != mini))
		break;
	}
	
	return meas[medi];
}

// Mean and Standard Deviation Module
void init_M_SD_module(uint8_t goal_num_samples)
{
	M_SD_target_num_samples = goal_num_samples;
	M_SD_num_samples = 0;
	M_SD_mean = 0;
	M_SD_variance = 0;
}

uint8_t M_SD_module_add_value(float val)
{	
	float k = M_SD_num_samples + 1;
	float next_mean;
	float next_var;

	// increment the mean:
	if(M_SD_num_samples == 0)
	{
		next_mean = val;
	}
	else
	{
		next_mean = (M_SD_mean*(k-1) + val)/k;
	}

	// increment the variance:
	if(M_SD_num_samples >= 2)	// k > 2 (third or higher value being added)
	{
		//next_var = ((k-2)/(k-1))*M_SD_variance + ((1/k)*(val-M_SD_mean)*(val-M_SD_mean))
		next_var = ((k-2)*M_SD_variance)/(k-1) + ((val-M_SD_mean)/k)*(val-M_SD_mean);
	}
	else if(M_SD_num_samples == 1)	// k == 2 (second value being added)
	{
		next_var = (val-M_SD_mean)*(val-M_SD_mean)/2.0;
	}
	else	// k == 1 (first value being added)
	{
		next_var = 0;
	}
	
	M_SD_mean = next_mean;
	M_SD_variance = next_var;
	M_SD_num_samples++;

	// alert user that the mean and variance are ready for sampling:
	if(M_SD_num_samples >= M_SD_target_num_samples)
		return 1;
	else
		return 0;
}

float M_SD_module_get_mean(void)
{
	return M_SD_mean;
}

float M_SD_module_get_stdev(void)
{
	return sqrt(M_SD_variance);
}


// ----- data printing operations:

void print_mathematica_formatted_2column(uint8_t x_points[], float y_points[], uint8_t num_points)
{
	printf("{");
	for(uint8_t i = 0; i < num_points; i++)
	{
		printf("\r\n{%u,%.6f},", x_points[i], (double)y_points[i]);
		_delay_ms(5);
	}
	printf("\b};\r\n");
}

void print_precise_line_equation(float slope, float yint)
{
	printf("line: y = %.6f+%.6f*x\r\n", yint, slope);
	
	_delay_ms(5);
}

// ----- debugging operations: 

void print_uint8_in_binary(uint8_t num)
{
	if(num < 10)		// single digit
		printf("  %u = ", num);
	else if(num < 100)	// two digit
		printf(" %u = ", num);
	else				// three digit
		printf("%u = ", num);
	
	for(int8_t i = 7; i >= 0; i--)
	{
		printf("%u", (num >> i) & 1);
		//_delay_ms(2);
	}
	
	printf("\r\n");
}

void compare_two_uint64_print(uint64_t num1, uint64_t num2)
{
	uint8_t byte;
	
	if(num1 == num2)	printf("SAME\r\n");
	else				printf("DIFFERENT\r\n");
	
	/*
	for(uint8_t byte = 0; byte < 8; byte++)
	{
		printf("%u: ", byte + 1);
		printf("%u, ", (num1>>(8*byte)) & 0xFF);
		printf("%u\r\n", (num2>>(8*byte)) & 0xFF);
		_delay_ms(5);
	}*/

	printf("1: ");
	for(byte = 8; byte > 0; byte--)
	{
		printf("%2x ", (num1>>(8*(byte-1))) & 0xFF);
		_delay_ms(2);
	}	printf("\r\n");
	printf("2: ");
	for(byte = 8; byte > 0; byte--)
	{
		printf("%2x ", (num2>>(8*(byte-1))) & 0xFF);
		_delay_ms(2);
	}	printf("\r\n");
}



void checkpoint_print(uint8_t symbol)
{
	if(ALLOW_CHECKPOINT_PRINT)
		printf("%c",symbol);
}

void halt_program()
{
	_delay_ms(50);

	// LEDs are monitoring interrupts, sometimes
	//set_LED(1, LED_SATURATION_LOW);
	//set_LED(2, LED_SATURATION_LOW);
	
	printf("HALTED, must reboot, debug your program\r\n");
	//fprintf_P(&usart_stream, PSTR("HALTED, must reboot, debug your program\r\n"));

	_delay_ms(20);

	cli();	//disable_interrupts

	while(1==1)
	{/* infinite fail loop */};
}

