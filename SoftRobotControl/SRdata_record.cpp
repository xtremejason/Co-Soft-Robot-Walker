// SRdata_record.cpp

#include "SRdata_record.h"

#include "SRpressure_control.h"
#include "SRtouch_control.h"

#include "../SoftRobotSource/SRstretch_sensor.h"
#include "../SoftRobotSource/SRpump.h"
#include "../SoftRobotSource/software_clock.h"
#include "../SoftRobotSource/SRPC_com.h"
#include "../SoftRobotSource/SRcapacitive_touch.h"
#include "../SoftRobotSource/SR9axis_IMU.h"

// private variables
bool recording = 0;					// lines are printed when set		

uint16_t record_lines = 0;			// keeps count of the number of lines printed, not used
uint16_t max_lines_to_record;		// used to limit the number of lines printed, some terminals have maximum line limit

float record_time;					// time in seconds since data recording began
uint16_t last_record_time_ms = 0;	// time in milliseconds, time that last data line was printed

uint8_t frame_skip_countdown;		// countdown iterator, line prints when == 0

uint8_t record_channel_flags = 0;	// keeps track of which actuators we are recording, e.g. 1,3,4 = 0b00001101
uint8_t record_cap_pad_flags = 0;

uint8_t frames_per_line = 1;
bool column_format;

#define DATA_HEADER_PRINT_DELAY_MS 3
#define COMMENT_CHAR '#'

// ------------------------------------------------------------------------------------------//
#define RECORD_TYPE_MAIN			0x00000011  // <-- CHANGE THIS TO WHAT YOU WANT RECORDED

#define RECORD_MAIN_PCB_PRESSURE	0x00000001	// MAIN PRESSURE	(PSI)
#define RECORD_MAIN_PUMP_STATUS		0x00000010	// PUMP_ON			(bool)

// ------------------------------------------------------------------------------------------//
#define RECORD_TYPE_ACTUATOR		0x00010110	// <-- CHANGE THIS TO WHAT YOU WANT RECORDED

#define RECORD_ACT_PRESSURE_ADC		0x00000001	// ANALOG PRESSURE	(ADC) 
#define RECORD_ACT_PRESSURE_PSI		0x00000010	// ANALOG PRESSURE	(PSI)
#define RECORD_ACT_SET_PRESSURE		0x00000100	// SET PRESSURE		(PSI)
#define RECORD_ACT_PRESS_ERROR		0x00001000	// PRESSURE ERROR	(PSI)
#define RECORD_ACT_STRAIN_ADC		0x00010000	// STRAIN SENSOR	(ADC)
#define RECORD_ACT_STRAIN_K			0x00100000	// STRAIN SENSOR	(1/m)
#define RECORD_VALVE_FILL			0x01000000	// FILL VALVE		(bool)
#define RECORD_VALVE_VENT			0x10000000	// VENT VALVE		(bool)

// ------------------------------------------------------------------------------------------//
#define RECORD_TYPE_CAPACITANCE		0x00000000	// <-- CHANGE THIS TO WHAT YOU WANT RECORDED
//#define RECORD_TYPE_CAPACITANCE 	0x00000001	// <-- CHANGE THIS TO WHAT YOU WANT RECORDED

#define RECORD_CAP_PROXIMITY_ADC	0x00000001	// 10-bit conv val	(ADC) 

// ------------------------------------------------------------------------------------------//
//#define RECORD_TYPE_IMU 			0x01100100	// <-- CHANGE THIS TO WHAT YOU WANT RECORDED
#define RECORD_TYPE_IMU 			0x00000000	// <-- CHANGE THIS TO WHAT YOU WANT RECORDED

#define RECORD_IMU_ACCEL_G			0x00000001	// 3D acceleration	(g's)
//#define RECORD_IMU_ACCEL_G_NORM	0x00000010	// |acceleration|	(g's)
#define RECORD_IMU_ACCEL_SI			0x00000100	// 3D acceleration	(m/s^2)
//#define RECORD_IMU_ACCEL_SI_NORM	0x00001000	// 3D acceleration	(m/s^2)
#define RECORD_IMU_GYRO_DEG			0x00010000	// angular rotation	(º/s)
#define RECORD_IMU_GYRO_SI 			0x00100000	// angular rotation	(rad/s)
#define RECORD_IMU_MAG_UT			0x01000000
// ------------------------------------------------------------------------------------------//


/* USER ACCESSIBLE FUNCTIONS */

void record_data(uint8_t record_mode)
{
	if(!recording)
	{
		recording = 1;

		// 'zero' out the recording clock
		record_time = 0;
		last_record_time_ms = get_milliseconds();	// global assignment
		record_lines = 0;
		//frame_skip_countdown = 0;
		frame_skip_countdown = 1;	// test, skip the first frame
		
		max_lines_to_record = MAXIMUM_LINES_TO_RECORD;

		request_overtime_permission();
		
		if(record_mode == RECORD_MODE_SIMPLE)
		{
			frames_per_line = NUM_FRAMES_PER_RECORD_LINE_SLOW;
			column_format = true;
		}
		else
		{
			frames_per_line = NUM_FRAMES_PER_RECORD_LINE_FAST;	// every frame, detailed record
			column_format = false;	// true CSV format
		}

		// print out recording settings (not part of record file)
		float line_delta_t;
		line_delta_t = frames_per_line*get_main_loop_period_s();
		printf("RECORD | %.3f s/line", line_delta_t);
		printf(" | max %u lines\r\n", MAXIMUM_LINES_TO_RECORD);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);

		
		printf("\r\n");	// skip a line

		if(verify_data_record_sensors_ready() > 0)
		{
			// halt recording, let user fix problem
			recording = 0;
		}

		if(record_mode & RECORD_MODE_HEADER_TAGS)
		{
			generate_data_record_comments();
			generate_data_record_header();
		}
	}
	else	// stop recording
	{	
		recording = 0;
		printf("\r\n");
	}
}

void edit_record_parameters(void)	// MAJOR TODO (TODO) 
{
	float num;
	
	printf("Edit Record Parameters:\r\n");
	
	printf("1) Frames per line: %u\r\n", frames_per_line);
	printf("0) Quit\r\n");

	num = type_a_number_blocking(&usart);

	printf("typed: %.2f\r\n", num);
}

void data_record_channel_on(uint8_t act_num)
{
	record_channel_flags |= (1 << (act_num-1));
}

void data_record_channel_off(uint8_t act_num)
{
	record_channel_flags &=~ (1 << (act_num-1));
}

void data_record_cap_pad_on(uint8_t pad_num)
{
	record_cap_pad_flags |= (1 << (pad_num-1));
}

void data_record_cap_pad_off(uint8_t pad_num)
{
	record_cap_pad_flags &=~ (1 << (pad_num-1));
}

void snapshot_data_record(void)
{
	request_overtime_permission();
	
	printf("\r\n-snapshot-\r\n");
	_delay_ms(10);

	data_service();
}


/* OPERATING-SYSTEM ACCESSIBLE FUNCTIONS */

void data_service(void)	// this is the PRIMARY FUNCTION for printing lines!
{
	// time	variables:
	uint16_t time_ms;
	uint16_t t_diff_ms;
	
	if(recording)
	{
		if(frame_skip_countdown == 0)
		{
			// timing:
			time_ms = get_milliseconds();
			t_diff_ms = compute_tdiff_ms(last_record_time_ms, time_ms);
			record_time += (float)t_diff_ms/1000.0;		// NDF verified that record_time is actually accurate over 1 minute
			last_record_time_ms = time_ms;	// global assignment
			

			// ---------------------------------------------------------------------------------------------------------

			// print the variables:
			start_record_line(column_format);

			if(RECORD_TYPE_ACTUATOR)
			{
				for(uint8_t act_num = 1; act_num < 7; act_num++)
				{
					if(flag_is_on(record_channel_flags, act_num))
					{
						continue_record_line_actuator_data(act_num, column_format);
					}
				}
			}
			
			if(RECORD_TYPE_CAPACITANCE)
			{
				continue_record_line_capacitance_data(column_format, CAP_VALUE_TYPE_ADC);
			}
			
			if(RECORD_TYPE_IMU)
			{
				continue_record_line_IMU_data(column_format, 1);
				continue_record_line_IMU_data(column_format, 2);
			}

			end_record_line();

			// ---------------------------------------------------------------------------------------------------------
			
			record_lines++;	// count it
			frame_skip_countdown = frames_per_line - 1;	// reset the countdown for the next line

			if(record_lines >= max_lines_to_record)
			{	
				printf("%u LINE MAX\r\n", max_lines_to_record);
				record_data(0);		// halt the recording
			}
		}
		else
			frame_skip_countdown--;
	}
}

uint8_t is_data_recording(void)
{
	return recording;
}






/* INTERNAL FUNCTIONS - NON-PRINTABLES */

uint8_t verify_data_record_sensors_ready(void)
{
	uint8_t error_status = 0;	// 0 is OK, 1 is ERROR

	if(RECORD_TYPE_CAPACITANCE)
		if(!verify_touch_existance())			error_status = 1;
	
	if(RECORD_TYPE_IMU)
	{
		if(!verify_IMU_accel_existance(1))		error_status = 2;
		if(!verify_IMU_accel_existance(2))		error_status = 3;
		if(!verify_IMU_gyro_existance(1))		error_status = 2;
		if(!verify_IMU_gyro_existance(2))		error_status = 3;
		if(!verify_IMU_mag_existance(1))		error_status = 2;
		if(!verify_IMU_mag_existance(2))		error_status = 3;
	}

	if(error_status > 0)
	{
		printf("Error Detected: %u\r\n", error_status);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);
	}

	return error_status;
}

/* INTERNAL FUNCTIONS - PRINTABLES */

void generate_data_record_header(void)
{
	uint8_t field_count = 1;
	
	// all record types record the time	// TIME (S)
	printf("& %u, time (s)\r\n", field_count);
	_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	
	if(RECORD_TYPE_MAIN)
	{
		field_count = data_record_subheader_main(field_count);
	}

	if(RECORD_TYPE_ACTUATOR)
	{
		field_count = data_record_subheader_actuator(field_count);
	}

	// (TODO) currently we assume that only one capacitance strip is available
	if(RECORD_TYPE_CAPACITANCE)
	{
		field_count = data_record_subheader_cap_touch(field_count);
	}

	if(RECORD_TYPE_IMU)
	{
		field_count = data_record_subheader_IMU(field_count, 1);
		field_count = data_record_subheader_IMU(field_count, 2);
	}
}

uint8_t data_record_subheader_main(uint8_t field_count)
{
	if(RECORD_TYPE_MAIN & RECORD_MAIN_PCB_PRESSURE)	// CENTER PRESSURE (PSI)
	{
		printf("& %u, digital pressure (PSI)\r\n", field_count);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}
	if(RECORD_TYPE_MAIN & RECORD_MAIN_PUMP_STATUS)		// PUMP_ON (boolean)
	{
		printf("& %u, pump on (bool)\r\n", field_count);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}

	return field_count;
}

uint8_t data_record_subheader_actuator(uint8_t field_count)
{
	uint8_t act_num;

	printf("# Actuator record flags:");
	print_uint8_in_binary(record_channel_flags);

	for(act_num = 1; act_num < 7; act_num++)
	{
		if(flag_is_on(record_channel_flags, act_num))
		{
			if(RECORD_TYPE_ACTUATOR & RECORD_ACT_PRESSURE_ADC)	// ANALOG PRESSURE (ADC)
			{
				printf("& %u, A%u pressure (ADC)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_ACT_PRESSURE_PSI)	// ANALOG PRESSURE (PSI)
			{
				printf("& %u, A%u pressure (PSI)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_ACT_SET_PRESSURE)		// SET PRESSURE (PSI)
			{
				printf("& %u, A%u set pressure (PSI)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_ACT_PRESS_ERROR)		// PRESSURE ERROR (PSI)
			{
				printf("& %u, A%u pressure error (PSI)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_ACT_STRAIN_ADC)		// STRAIN SENSOR (ADC)
			{
				printf("& %u, A%u strain (ADC)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_ACT_STRAIN_K)		// STRAIN SENSOR (1/m)
			{
				printf("& %u, A%u curvature (1/m)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_VALVE_FILL)			// FILL VALVE (boolean)
			{
				printf("& %u, A%u fill valve (bool)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
			if(RECORD_TYPE_ACTUATOR & RECORD_VALVE_VENT)			// VENT VALVE (boolean)
			{
				printf("& %u, A%u vent valve (bool)\r\n", field_count, act_num);
				_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
			}
		}
	}
	
	return field_count;
}

uint8_t data_record_subheader_cap_touch(uint8_t field_count)
{
	uint8_t pad_num;
	
	for(pad_num = 1; pad_num <= get_num_touch_pads(); pad_num++)
	{
		if(flag_is_on(record_cap_pad_flags, pad_num))
		{
			printf("& %u, TouchPad-%u valve\r\n", field_count, pad_num);
			_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		}
	}

	return field_count;
}

uint8_t data_record_subheader_IMU(uint8_t field_count, uint8_t IMUnum)
{
	if(RECORD_TYPE_IMU & RECORD_IMU_ACCEL_G)
	{
		printf("& %u, x-accel-%u (g's)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, y-accel-%u (g's)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, z-accel-%u (g's)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}
	/*if(RECORD_TYPE_IMU & RECORD_IMU_ACCEL_NORM)
	{
		printf("& %u, |accel| (float)\r\n", field_count);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}*/
	if(RECORD_TYPE_IMU & RECORD_IMU_ACCEL_SI)
	{
		printf("& %u, x-accel-%u (m/s^2)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, y-accel-%u (m/s^2)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, z-accel-%u (m/s^2)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}
	
	if(RECORD_TYPE_IMU & RECORD_IMU_GYRO_DEG)
	{
		printf("& %u, x-gyro-%u (ø/s)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, y-gyro-%u (ø/s)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, z-gyro-%u (ø/s)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}
	if(RECORD_TYPE_IMU & RECORD_IMU_GYRO_SI)
	{
		printf("& %u, x-gyro-%u (rad/s)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, y-gyro-%u (rad/s)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, z-gyro-%u (rad/s)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}

	if(RECORD_TYPE_IMU & RECORD_IMU_MAG_UT)
	{
		printf("& %u, x-mag-%u (uT)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, y-mag-%u (uT)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
		printf("& %u, z-mag-%u (uT)\r\n", field_count, IMUnum);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);	field_count++;
	}

	return field_count;
}


void generate_data_record_comments(void)
{
	if(RECORD_TYPE_CAPACITANCE)
	{
		printf("%c sensor strip ver %u\r\n", COMMENT_CHAR, SENSOR_STRIP_VERSION);
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);
		
		printf("%c charge I %u uA\r\n", COMMENT_CHAR, get_MPR121_CDC_uA());
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);

		printf("%c charge T %.1f us\r\n", COMMENT_CHAR, get_MPR121_CDT_us());
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);

		printf("%c charge Q %.1f pC\r\n", COMMENT_CHAR, get_MPR121_applied_charge_pC());
		_delay_ms(DATA_HEADER_PRINT_DELAY_MS);
	}

	if(RECORD_TYPE_IMU)
	{
		for(uint8_t IMU_num = 1; IMU_num <= 2; IMU_num++)
		{
			printf("%c IMU-%u\r\n", COMMENT_CHAR, IMU_num);
			_delay_ms(DATA_HEADER_PRINT_DELAY_MS);
			
			printf("%c Accelerometer resolution: %.1f g\r\n", COMMENT_CHAR, get_9axis_accel_resolution(IMU_num));
			_delay_ms(DATA_HEADER_PRINT_DELAY_MS);

			printf("%c Gyroscope resolution: %.1f ø/s\r\n", COMMENT_CHAR, get_9axis_gyro_resolution(IMU_num));
			_delay_ms(DATA_HEADER_PRINT_DELAY_MS);

			printf("%c Magnetometer resolution: %.1f uT\r\n", COMMENT_CHAR, get_9axis_mag_resolution(IMU_num));
			_delay_ms(DATA_HEADER_PRINT_DELAY_MS);
		}
	}
}

void start_record_line(uint8_t columns)
{
	// regular record variables:	(should print in this order!)
	float center_pressure;		// MAIN PRESSURE
	bool pump_on_status;		// BOOLEAN
	
	// all record types record the time	// TIME (S)
	if(columns && record_time >= 100)		printf("%.1f", (double)record_time);
	else if(columns && record_time >= 10)	printf("%.2f", (double)record_time);
	else									printf("%.3f", (double)record_time);

	// collect the main/regular variables:
	center_pressure = get_digital_pressure();
	if(center_pressure < 0)
		center_pressure *= -1.0;	// sign correction, to avoid printing -0.00
	pump_on_status = get_pump_is_on();

	// print the main variables:
	if(RECORD_TYPE_MAIN & RECORD_MAIN_PCB_PRESSURE)		// CENTER PRESSURE (PSI)
		printf(",%.2f", (double)center_pressure);
	if(RECORD_TYPE_MAIN & RECORD_MAIN_PUMP_STATUS)		// PUMP_ON (boolean)
		printf(",%u", pump_on_status);
}

void end_record_line(void)
{
	printf("\r\n");
}


void continue_record_line_actuator_data(uint8_t act_num, uint8_t columns)
{
	// actuator record variables:	(should print in this order!)
	uint8_t p_meas_ADC;			// ANALOG PRESSURE (ADC)
	float p_meas_PSI;			// ANALOG PRESSURE (PSI)
	uint8_t s_meas_ADC;			// ANALOG STRAIN
	float s_meas_curv;			// ESTIMATED CURVATURE
	float p_set_PSI;
	float p_error_PSI;
	bool fill_valve_open;		// BOOLEAN
	bool vent_valve_open;		// BOOLEAN
	
	// collect the actuator variables:
	p_meas_ADC = get_actuator_pressure_last_ADC(act_num);
	p_meas_PSI = get_actuator_pressure_PSI(act_num);
	s_meas_ADC = get_ADC_stretch(act_num);
	s_meas_curv = get_strain_curvature(act_num);
	p_set_PSI = get_target_pressure(act_num);
	p_error_PSI = p_meas_PSI - p_set_PSI;
	fill_valve_open = fill_valve_is_open(act_num);
	vent_valve_open = vent_valve_is_open(act_num);
						
	if(columns)
		printf(" |");
											
	// print the actuator variables:
	if(RECORD_TYPE_ACTUATOR & RECORD_ACT_PRESSURE_ADC)	// ANALOG PRESSURE (ADC)
	{	if(columns)	printf(" ");	else printf(",");	printf("%u", p_meas_ADC);}
	if(RECORD_TYPE_ACTUATOR & RECORD_ACT_PRESSURE_PSI)	// ANALOG PRESSURE (PSI)
	{	if(columns)	printf(" ");	else printf(",");	printf("%.2f", (double)p_meas_PSI);}
	if(RECORD_TYPE_ACTUATOR & RECORD_ACT_SET_PRESSURE)	// SET PRESSURE (PSI)
	{	if(columns)	printf(" ");	else printf(",");	printf("%.2f", (double)p_set_PSI);}
	if(RECORD_TYPE_ACTUATOR & RECORD_ACT_PRESS_ERROR)	// PRESSURE ERROR (PSI)
	{	if(columns)	printf(" ");	else printf(",");	printf("%.2f", (double)p_error_PSI);}
	if(RECORD_TYPE_ACTUATOR & RECORD_ACT_STRAIN_ADC)	// STRAIN SENSOR (ADC)
	{	if(columns)	printf(" ");	else printf(",");	printf("%u", s_meas_ADC);}
	if(RECORD_TYPE_ACTUATOR & RECORD_ACT_STRAIN_K)		// STRAIN SENSOR (1/m)
	{	if(columns)	printf(" ");	else printf(",");	printf("%.2f", (double)s_meas_curv);}
	if(RECORD_TYPE_ACTUATOR & RECORD_VALVE_FILL)			// FILL VALVE (boolean)
	{	if(columns)	printf(" ");	else printf(",");	printf("%u", fill_valve_open);}
	if(RECORD_TYPE_ACTUATOR & RECORD_VALVE_VENT)			// VENT VALVE (boolean)
	{	if(columns)	printf(" ");	else printf(",");	printf("%u", vent_valve_open);}				
}

void continue_record_line_capacitance_data(uint8_t columns, uint8_t type)
{
	float cap_values[MAX_NUM_TOUCH_PADS];
	uint8_t num_vals;
	uint8_t pad_num;

	// collect the actuator variables:
	//num_vals = get_C_ADC_values_all_pads(cap_values);

	//num_vals = get_cap_values_allpads(cap_values, CAP_VALUE_TYPE_ADC);
	num_vals = get_cap_values_allpads(cap_values, type, 0);

	if(RECORD_TYPE_CAPACITANCE & RECORD_CAP_PROXIMITY_ADC)
	{
		for(uint8_t i = 0; i < num_vals; i++)
		{
			pad_num = i+1;
			if(flag_is_on(record_cap_pad_flags, pad_num))
			{
				if(columns)	printf(" ");	else printf(",");
			
				if(type == CAP_VALUE_TYPE_CAPACITANCE)	printf("%.3f", cap_values[i]);
				else									printf("%.0f", cap_values[i]);
			}
		}
	}
}

void continue_record_line_IMU_data(uint8_t columns, uint8_t IMU_num)
{
	// TODO: some of these could be nested
	
	float accel[3];
	float gyro[3];
	float mag[3];
	uint8_t i;

	if(RECORD_TYPE_IMU & RECORD_IMU_ACCEL_G)
	{
		get_9axis_accel_g(accel, 0, IMU_num);

		for(i = 0; i < 3; i++)
		{
			if(columns)	printf(" ");	else printf(",");
			printf("%.4f", accel[i]);
		}
	}

	if(RECORD_TYPE_IMU & RECORD_IMU_ACCEL_SI)
	{
		get_9axis_accel_SI(accel, IMU_num);

		for(i = 0; i < 3; i++)
		{
			if(columns)	printf(" ");	else printf(",");
			printf("%.4f", accel[i]);
		}
	}

	if(RECORD_TYPE_IMU & RECORD_IMU_GYRO_DEG)
	{
		get_9axis_gyro_deg(gyro, IMU_num);

		for(i = 0; i < 3; i++)
		{
			if(columns)	printf(" ");	else printf(",");
			printf("%.4f", gyro[i]);
		}
	}

	if(RECORD_TYPE_IMU & RECORD_IMU_GYRO_SI)
	{
		get_9axis_gyro_SIrad(gyro, IMU_num);

		for(i = 0; i < 3; i++)
		{
			if(columns)	printf(" ");	else printf(",");
			printf("%.4f", gyro[i]);
		}
	}

	if(RECORD_TYPE_IMU & RECORD_IMU_MAG_UT)
	{
		get_9axis_mag_uT(mag, IMU_num);

		for(i = 0; i < 3; i++)
		{
			if(columns)	printf(" ");	else printf(",");
			printf("%.1f", mag[i]);
		}
	}
}