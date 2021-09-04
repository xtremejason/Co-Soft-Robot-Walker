// SRdata_record.h

#ifndef __SRDATA_RECORD_H__
#define __SRDATA_RECORD_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"
#include <stdio.h>
#include <stdlib.h>		// for bool


#define	RECORD_MODE_SIMPLE			0	// 0 -> do not print a header
#define RECORD_MODE_HEADER_TAGS		1// 1 -> begin datastream with header tags
#define RECORD_MODE_HEADER_COMMENTS	2// 2 -> begin datastream with header comments
#define RECORD_MODE_FULL_HEADER		3 // 3 -> begin datastream with header comments AND tags


// the frame frequency is tied to the main loop frequency, currently 40 ms
// a 'frame' is defined as 1 pass through the main loop
// note: MAIN_LOOP_TIMEOUT_ms defined in software_clock.h, governs the time it takes for the 'main' loop
#define NUM_FRAMES_PER_RECORD_LINE_SLOW 		4	// must be >= 1
//#define NUM_FRAMES_PER_RECORD_LINE_FAST		1	// must be >= 1
#define NUM_FRAMES_PER_RECORD_LINE_FAST 		2	// must be >= 1	(temporary)

#define MAXIMUM_LINES_TO_RECORD		9900	// Tera Term set to 10000

/* USER ACCESSIBLE FUNCTIONS */

// Start/Stop sending the sensor readings to the serial port
void record_data(uint8_t header_type);
// header_type:
// 0 -> do not print a header
// 1 -> begin datastream with header tags
// 2 -> begin datastream with header comments
// 3 -> begin datastream with header comments AND tags

// MENU-style function to allow the user to turn on/off recordable features
void edit_record_parameters(void);
// NOT FULLY IMPLEMENTED! (TODO)

// turn on an actuator channel for recording: 1 - 6
void data_record_channel_on(uint8_t act_num);

// turn off an actuator channel for recording: 1 - 6
void data_record_channel_off(uint8_t act_num);

// turn on a captouch pad for recording: 1 - 8
void data_record_cap_pad_on(uint8_t pad_num);

// turn off a captouch pad for recording: 1 - 8
void data_record_cap_pad_off(uint8_t pad_num);

void snapshot_data_record(void);


/* OPERATING-SYTEM ACCESSIBLE FUNCTIONS */

// called in main loop to print a line of data to serial port
void data_service(void);
// note: a data line is not [likely] printed every call
//	- see comments about NUM_FRAMES_PER_RECORD_LINE for more info

// get function, boolean
uint8_t is_data_recording(void);
// this is not currently called anywhere


/* INTERNAL FUNCTIONS - NON-PRINTABLES */

// check if all the sensors that will be contributing values are responding
uint8_t verify_data_record_sensors_ready(void);
// return:	1 if everything is OK
//			0 if FAIL (debug required) 

/* INTERNAL FUNCTIONS - PRINTABLES */

// print the header for the start of a data record
void generate_data_record_header(void);

uint8_t data_record_subheader_main(uint8_t field_count);

uint8_t data_record_subheader_actuator(uint8_t field_count);

uint8_t data_record_subheader_cap_touch(uint8_t field_count);

uint8_t data_record_subheader_IMU(uint8_t field_count, uint8_t IMUnum);

// print the comments (optional) start of data record
void generate_data_record_comments(void);

// print the start of a new record line
void start_record_line(uint8_t columns);

// print the end of a record line (endline symbol)
void end_record_line(void);

// print the continuation of a record line, actuator parameters
void continue_record_line_actuator_data(uint8_t act_num, uint8_t columns);
// act_num: 1 - 6
// columns: boolean, TRUE if data should print in clear columns, FALSE if CSV is sufficient

// print the continuation of a record line, capacitive touch parameters
void continue_record_line_capacitance_data(uint8_t columns, uint8_t type);

void continue_record_line_IMU_data(uint8_t columns, uint8_t IMUnum);


#endif