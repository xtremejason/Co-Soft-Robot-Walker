// SRmisc_functions.h

#ifndef __SRMISC_FUNCTIONS_H__
#define __SRMISC_FUNCTIONS_H__

#include <avr/io.h>


#define ALLOW_CHECKPOINT_PRINT  	0
//#define ALLOW_CHECKPOINT_PRINT  	1


#ifdef __cplusplus
extern "C"
{
#endif

// ----- bitwise operations:

// boolean, check to see if a bit-flag is set in a byte
// note flag 1 = 0b00000001, flag 8 = 0b10000000
uint8_t flag_is_on(uint8_t flags, uint8_t flag_num);

// used to make a byte of individual flags
// pass in a byte of flags and a number representing which flag to turn on
// the byte of flags will be returned with the new flag on
// call this function repeatedly on the same byte to set multiple flags
uint8_t make_flag(uint8_t prior_flags, uint8_t flag_num);

// ----- bytewise operations:

uint32_t build_uint32_from_byte_array(uint8_t bytes[], uint8_t num_bytes);

// convert arbitrary bit representation into its signed value in 2's complement
int16_t twos_complement_conversion(uint16_t unsigned_value, uint8_t sign_bit_pos);
// example:	 unsigned_value = 254,	sign_bit_pos = 7 | output = -2
// example:	 unsigned_value = 5,	sign_bit_pos = 2 | output = -3
// sign_bit_pos always begins with 0, e.g. MSB in a byte is bit 7

// ----- unit conversions:

float kPa_to_PSI(float kPa);

float mbar_to_PSI(float mbar);

float celsius_to_farenheit(float celsius);

float farenheit_to_celsius(float farenheit);

// ----- math operations:

// ordinary least squares equation solver
// input: a sequence of {x,y} coordinate pairs
// output: a line representing the best fit through the points
void OLS_solver(uint8_t x_points[], float y_points[], uint8_t num_points, float *slope_soln, float *yint_soln);

// function for getting the median value from an array of 3 values (uint8_t)
uint8_t find_median_of_3(uint8_t* meas);

// Mean and Standard Deviation Module
void init_M_SD_module(uint8_t goal_num_samples);
// input: number of samples to collect in order to report that mean/SD are ready 

uint8_t M_SD_module_add_value(float val);
// return: true when goal_num_samples have been collected, false o/w

float M_SD_module_get_mean(void);

float M_SD_module_get_stdev(void);




// ----- data printing operations:

// prints a list of 2d-points {x,y} in such a way that it can be copy-pasted
// into Mathematica for quick plotting of data
void print_mathematica_formatted_2column(uint8_t x_points[], float y_points[], uint8_t num_points);

// prints an equation of the form: y = m*x + b
// note: slope = m, yint = b
void print_precise_line_equation(float slope, float yint);

// ----- debugging operations: 

// print unsigned integers as their bitwise representation
// e.g. prints: " 88 = 01011000\r\n"
void print_uint8_in_binary(uint8_t num);

void compare_two_uint64_print(uint64_t num1, uint64_t num2);

// generic toggle-able DEBUG print, can be used anywhere in the code
// symbol is any char, use single quote characters: 'a'
void checkpoint_print(uint8_t symbol);

// can be called to deliberately crash the program (typically for debugging purposes)
void halt_program(void);

#ifdef __cplusplus
}
#endif

#endif