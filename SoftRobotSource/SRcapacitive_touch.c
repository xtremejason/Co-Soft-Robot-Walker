//SRcapacitive_touch.c

#include "SRcapacitive_touch.h"

#include <util/delay.h>
#include <stdio.h>			// used for printf_P
#include <avr/pgmspace.h>	// used for PSTR

#include "software_clock.h"	// for overtime permission
#include "SRmisc_functions.h"

#define TOUCH_DEBUGMODE 	0	// no printing
//#define TOUCH_DEBUGMODE 	1	// prints when baseline values are out of range
//#define TOUCH_DEBUGMODE 	2	// prints when data is recycled and when new baselines are collected

#define MPR121_ADC_VMEAS_MAX	807		// (3.3-0.7)/3.3 * 1024 = 806.7878..
#define MPR121_ADC_VMEAS_MIN	217		// 0.7/3.3 * 1024 = 217.2121..

/* MPR121 Register Defines */

// Touch Status Registers (0x00 - 0x01)

// Out Of Range Status Registers (0x02 - 0x03)

// Electrode Data Registers	(0x04 - 0x1D)
// these registers are read-only
// these registers are updated every ESI x SFI
#define CAP_ELEC_0		0x04
#define CAP_ELEC_1		0x06
#define CAP_ELEC_2		0x08
#define CAP_ELEC_3		0x0A
#define CAP_ELEC_4		0x0C
#define CAP_ELEC_5		0x0E
#define CAP_ELEC_6		0x10
#define CAP_ELEC_7		0x12
#define CAP_ELEC_8		0x14
#define CAP_ELEC_9		0x16
#define CAP_ELEC_10		0x18
#define CAP_ELEC_11		0x1A

// Electrode Baseline Values (0x1E - 0x2A)
#define BV_ELEC_0		0x1E
#define BV_ELEC_1		0x1F
#define BV_ELEC_2		0x20
#define BV_ELEC_3		0x21
#define BV_ELEC_4		0x22
#define BV_ELEC_5		0x23
#define BV_ELEC_6		0x24
#define BV_ELEC_7		0x25
#define BV_ELEC_8		0x26
#define BV_ELEC_9		0x27
#define BV_ELEC_10		0x28
#define BV_ELEC_11		0x29

// Baseline Filtering Control Registers (0x2B - 0x40)
#define MHD_R		0x2B	// Maximum Half Delta		(1 - 63)
#define NHD_R		0x2C	// Noise Half Delta			(1 - 63)
#define	NCL_R 		0x2D	// Noise Count Limit		(0 - 255)
#define	FDL_R		0x2E	// Filter Delay Count Limit	(0 - 255)
#define	MHD_F		0x2F	// Maximum Half Delta		(1 - 63)
#define	NHD_F		0x30	// Noise Half Delta			(1 - 63)
#define	NCL_F		0x31	// Noise Count Limit		(0 - 255)
#define	FDL_F		0x32	// Filter Delay Count Limit	(0 - 255)
// Note: '13th channel' ELEPROX is controlled by registers 0x36 - 0x40

// Touch and Release Thresholds (0x41 - 0x5A)
#define	ELE0_T		0x41
#define	ELE0_R		0x42
#define	ELE1_T		0x43
#define	ELE1_R		0x44
#define	ELE2_T		0x45
#define	ELE2_R		0x46
#define	ELE3_T		0x47
#define	ELE3_R		0x48
#define	ELE4_T		0x49
#define	ELE4_R		0x4A
#define	ELE5_T		0x4B
#define	ELE5_R		0x4C
#define	ELE6_T		0x4D
#define	ELE6_R		0x4E
#define	ELE7_T		0x4F
#define	ELE7_R		0x50
#define	ELE8_T		0x51
#define	ELE8_R		0x52
#define	ELE9_T		0x53
#define	ELE9_R		0x54
#define	ELE10_T		0x55
#define	ELE10_R		0x56
#define	ELE11_T		0x57
#define	ELE11_R		0x58

// AFE Configuration Registers (0x5C - 0x5D)
#define AFE_CONFIG_1	0x5C
#define	AFE_CONFIG_2	0x5D

// Electrode Configuration Register (0x5E)
#define	ELE_CFG		0x5E

// Individual Charge/Discharge Current Registers (0x5F - 0x6B)
// non-zero values override global CDC setting (AFE Config 1 Register 0x00XXXXXX)
#define CDC_ELEC_0	0x5F
#define CDC_ELEC_1	0x60
#define CDC_ELEC_2	0x61
#define CDC_ELEC_3	0x62
#define CDC_ELEC_4	0x63
#define CDC_ELEC_5	0x64
#define CDC_ELEC_6	0x65
#define CDC_ELEC_7	0x66
#define CDC_ELEC_8	0x67
#define CDC_ELEC_9	0x68
#define CDC_ELEC_10	0x69
#define CDC_ELEC_11	0x6A

// Individual Charge/Discharge Time Registers (0x6C - 0x72)
// non-zero values override global CDT setting (AFE Config 2 Register 0xXXX00000)
#define CDT_ELEC_0	0x6C	// low-nibble
#define CDT_ELEC_1	0x6C	// high-nibble
#define CDT_ELEC_2	0x6D	// low-nibble
#define CDT_ELEC_3	0x6D	// high-nibble
#define CDT_ELEC_4	0x6E	// low-nibble
#define CDT_ELEC_5	0x6E	// high-nibble
#define CDT_ELEC_6	0x70	// low-nibble
#define CDT_ELEC_7	0x70	// high-nibble
#define CDT_ELEC_8	0x71	// low-nibble
#define CDT_ELEC_9	0x71	// high-nibble
#define CDT_ELEC_10	0x72	// low-nibble
#define CDT_ELEC_11	0x72	// high-nibble

// GPIO Registers (0x73 - 0x7A)
#define GPIO_CTRL0	0x73
#define	GPIO_CTRL1	0x74
#define GPIO_DATA	0x75
#define	GPIO_DIR	0x76
#define	GPIO_EN		0x77
#define	GPIO_SET	0x78
#define	GPIO_CLEAR	0x79
#define	GPIO_TOGGLE	0x7A

// Auto Configuration Registers (0x7B - 0x7F)
#define	AUTO_CFG_0		0x7B
#define AUTO_CFG_1		0x7C
#define	AUTO_CFG_USL	0x7D
#define	AUTO_CFG_LSL	0x7E
#define	AUTO_CFG_TL		0x7F

// Software Reset Register (0x80)
#define SOFT_RST		0x80

// default settings
//#define CDC_DEFAULT_BITS	60	// charge-discharge current (hardware default 16)
//#define CDT_DEFAULT_BITS	1	// charge-discharge time (hardware default 1)
//#define CDC_DEFAULT_BITS	25	// charge-discharge current (hardware default 16)
//#define CDT_DEFAULT_BITS	3	// charge-discharge time (hardware default 1)
#define CDC_DEFAULT_BITS	36	// charge-discharge current (hardware default 16)
#define CDT_DEFAULT_BITS	2	// charge-discharge time (hardware default 1)



/* PRIVATE VARIABLES */

uint8_t MPR121_address;		// temporary, there could be up to 4 on the same bus!

// Analog Front End (AFE) Configuration Settings:
uint8_t first_filter_iterations;			// [number of samples]	range: 6 - 34
uint8_t charge_discharge_current;			// [microamperes]		range: 1 - 63 uA
float charge_discharge_time;				// [microseconds]		range: 0.5 - 32 us
uint8_t second_filter_iterations;			// [number of samples]	range: 4 - 18
uint8_t electrode_sampling_interval;		// [milliseconds]		range: 1 - 128 ms
uint16_t MPR121_meas_update_interval_ms;	// [milliseconds]		range: 4 - 2304 ms (ESI x SFI)

// Electrode Configuration Settings:
uint8_t num_individual_electrodes_enabled;	// range: 0 - 12
uint8_t num_proximity_electrodes_enabled;	// range: 0,2,4,12
uint8_t ecr_pause_value;					// for pause/resume of 'run mode'
uint8_t electrode_configuration_register;	// keep a copy of the ELE_CFG register, update only in touch_write()
uint8_t analog_front_end_register_1;		// keep a copy of the AFE_CONFIG_1 register, update only in touch_write()
uint8_t analog_front_end_register_2;		// keep a copy of the AFE_CONFIG_2 register, update only in touch_write()
uint8_t AFE_settings_updated;				// boolean, set only in touch_write()

// Touch Library Private Variables:
uint8_t touch_existance_confirmed;			// sorta-boolean, set in init if MPR121 found on I2C line
uint16_t last_touch_meas_time_ms;			// time of last measurement
uint16_t previous_C_ADC_vals[MPR121_NUM_ELECTRODES];	// most recent MPR121 ADC values
uint16_t baseline_C_ADC_vals[MPR121_NUM_ELECTRODES];	// always keeps the highest seen ADC values
														// [*note: passing this array to get_electrodes_values(_,_) will force a new baseline collection]


/* USER ACCESSIBLE FUNCTIONS */

uint8_t get_electrodes_values(uint16_t *cap_values, uint8_t num_electrodes)
{
	uint8_t e_i;	// electrode index
	uint16_t ADC_vals[MPR121_NUM_ELECTRODES];	// always allocate a full array
	//uint16_t ADC_vals[num_electrodes];
	uint8_t new_values_collected = 0;	// boolean

	if(verify_touch_existance())
	{
		// (3) CONDITIONS FOR COLLECTING A NEW SAMPLE:
		// CHECK 1: Are the analog front end settings changed?
		if(AFE_settings_updated)
		{
			//	waste two reads to continue
			touch_read_electrodes(MPR121_NUM_ELECTRODES, ADC_vals);
			touch_read_electrodes(MPR121_NUM_ELECTRODES, ADC_vals);

			AFE_settings_updated = 0;	// clear the flag for AFE update
			new_values_collected = 1;
		}
		// CHECK 2: Is the request to save a new baseline value?
		else if(cap_values == baseline_C_ADC_vals)
		{
			if(TOUCH_DEBUGMODE) printf("*blr*");
			
			// num_electrodes = MPR121_NUM_ELECTRODES; (ENABLE if want to override the length given for baseline collection)
			request_overtime_permission();
			_delay_ms(60);

			new_values_collected = 1;
		}
		// CHECK 3: Is the most recent collected data now stale?
		else if(!touch_data_is_fresh())	// 3]
		{
			new_values_collected = 1;
		}
		
		// NOW COLLECT NEW SAMPLE IF NEEDED, OTHERWISE USE PREVIOUS SAMPLE
		if(new_values_collected == 1)
		{
			touch_read_electrodes(12, ADC_vals);	// read all the electrode registers in one request

			//for(ei = 0; ei < MPR121_NUM_ELECTRODES; ei++)
			for(e_i = 0; e_i < num_electrodes; e_i++)
			{
				// save a copy of the values to a private array
				previous_C_ADC_vals[e_i] = ADC_vals[e_i];
				
				// populate the return variable
				cap_values[e_i] = ADC_vals[e_i];

				// update the baseline values if necessary *NEW*
				if(baseline_C_ADC_vals[e_i] < ADC_vals[e_i])
				baseline_C_ADC_vals[e_i] = ADC_vals[e_i];
			}

			// record the time of the data collection
			last_touch_meas_time_ms = get_milliseconds();
		}
		else	// simply fill in the return array with the last read values
		{
			//for(ei = 0; ei < MPR121_NUM_ELECTRODES; ei++)
			for(e_i = 0; e_i < num_electrodes; e_i++)
			{
				cap_values[e_i] = previous_C_ADC_vals[e_i];
			}

			// DEBUG PRINT HERE IF WANT TO KNOW WHEN A REQUEST HAPPENS
		}
	}
	
	// else should return array of 0s, since the values are not updated
	else
	{
		for(e_i = 0; e_i < num_electrodes; e_i++)
		{
			cap_values[e_i] = 0;
		}
	}

	
	
	return new_values_collected;
}

void get_electrodes_baseline(uint16_t *cap_values, uint8_t num_electrodes)
{
	uint8_t e_i;	// electrode index
	
	for(e_i = 0; e_i < num_electrodes; e_i++)
	{
		cap_values[e_i] = baseline_C_ADC_vals[e_i];
	}
}


/* USER ACCESSIBLE FUNCTIONS - CONVERSIONS */

float C_ADC_to_capacitance(uint16_t C_ADC)
{
	float capacitance;		// units: [C] = [I]*[T]/[V] = uA*us/V = pF
	
	capacitance = charge_discharge_time*charge_discharge_current/C_ADC_to_voltage(C_ADC);
	
	return capacitance;
}

float C_ADC_to_voltage(uint16_t C_ADC)
{
	float v_meas;
	
	v_meas = ((float)C_ADC/1024.0)*MPR121_VDD;
	
	return v_meas;
}


/* USER ACCESSIBLE FUNCTIONS - MPR121 SETTINGS */

uint8_t get_MPR121_CDC_uA(void)
{
	return charge_discharge_current;
}

float get_MPR121_CDT_us(void)
{
	return charge_discharge_time;
}

float get_MPR121_applied_charge_pC(void)
{
	float Q;	// units: [picocoulombs]

	Q = (charge_discharge_current) * (charge_discharge_time);

	return Q;
}


/* DEBUG FUNCTIONS - PRINT ONLY! */

void print_electrode_values(void)
{
	uint8_t ei;	// electrode index
	uint16_t ADC_vals[MPR121_NUM_ELECTRODES];

	get_electrodes_values(ADC_vals, MPR121_NUM_ELECTRODES);

	for(ei = 0; ei < MPR121_NUM_ELECTRODES; ei++)
	{
		printf("%u: ", ei);
		printf("%u\r\n", ADC_vals[ei]);
		_delay_ms(2);	// print used for debugging (longer delay than normal)
	}

}

void MPR121_debug_printout1(void)
{
	uint8_t data_val;
	
	data_val = touch_read(AFE_CONFIG_1);
	printf("AFE1 = 0x%02x =", data_val);
	print_uint8_in_binary(data_val);

	data_val = touch_read(AFE_CONFIG_2);
	printf("AFE2 = 0x%02x =", data_val);
	print_uint8_in_binary(data_val);

	data_val = touch_read(ELE_CFG);
	printf("ECR  = 0x%02x =", data_val);
	print_uint8_in_binary(data_val);
}

void MPR121_debug_printout2(void)
{
	uint8_t I;
	float T;
	float Q;
	
	I =  get_MPR121_CDC_uA();
	T = get_MPR121_CDT_us();
	Q =  get_MPR121_applied_charge_pC();

	printf("I = %u uA\r\n", I);
	printf("T = %.1f us\r\n", T);
	printf("Q = %.1f pC\r\n", Q);
	_delay_ms(5);
}

void MPR121_debug_printout3(void)
{
	uint16_t C_ADC_vals[MPR121_NUM_ELECTRODES];
	uint8_t ei;	// electrode index

	get_electrodes_values(C_ADC_vals, MPR121_NUM_ELECTRODES);

	_delay_ms(5);

	for(ei = 0; ei < MPR121_NUM_ELECTRODES; ei++)
	{
		printf("E%u: ", ei);
		printf("%u ", C_ADC_vals[ei]);
		printf("(%.3f pF)\r\n", C_ADC_to_capacitance(C_ADC_vals[ei]));
		_delay_ms(5);
	}
}



/* INTERNAL FUNCTIONS - HIGH LEVEL */

uint8_t electrode_register_addr(uint8_t electrode_index)
{
	//	this is the function we reproduce (from datasheet)
	//		#define CAP_ELEC_0		0x04
	//		...	(there are 2-bytes for each electrode)
	//		#define CAP_ELEC_11		0x1A

	uint8_t	register_address;

	register_address = 0x04 + 0x02*electrode_index;

	return register_address;
}

uint8_t check_touch_exists(void)
{
	uint8_t found = 0;
	
	if(I2C_check_device_exists(MPR121_I2C_ADDR_PIN_ON_GND))
	{
		MPR121_address = MPR121_I2C_ADDR_PIN_ON_GND;
		found = make_flag(found, 1);
	}
	else if(I2C_check_device_exists(MPR121_I2C_ADDR_PIN_ON_VCC))
	{
		MPR121_address = MPR121_I2C_ADDR_PIN_ON_VCC;
		found = make_flag(found, 2);
	}
	else if(I2C_check_device_exists(MPR121_I2C_ADDR_PIN_ON_SDA))
	{
		MPR121_address = MPR121_I2C_ADDR_PIN_ON_SDA;
		found = make_flag(found, 3);
	}
	else if(I2C_check_device_exists(MPR121_I2C_ADDR_PIN_ON_SCL))
	{
		MPR121_address = MPR121_I2C_ADDR_PIN_ON_SCL;
		found = make_flag(found, 4);
	}

	return found;
}

uint8_t verify_touch_existance(void)
{
	if(!touch_existance_confirmed)
	{
		printf("TOUCH DOES NOT EXIST\r\n");
		_delay_ms(5);	// print delay
	}
	
	return touch_existance_confirmed;
}

uint8_t touch_data_is_fresh(void)
{
	uint16_t current_time = get_milliseconds();
	uint16_t t_diff = compute_tdiff_ms(last_touch_meas_time_ms, current_time);
	
	
	if((t_diff <= MPR121_meas_update_interval_ms)&&(TOUCH_DEBUGMODE >= 2))
	{
		printf("*-%u\r\n", t_diff);
	}

	if(t_diff > MPR121_DATA_EXPIRATION_TIME_ms)
	{
		return 0;
	}

	return 1;
}


/* INTERNAL FUNCTIONS - LOW-LEVEL - ANALOG FRONT END SETTINGS */

uint8_t set_MPR121_first_filter_iterations_bits(uint8_t set_bits_2)
{
	// units: [iterations (number)] (6 is the default, range {6,10,18,34})
	// NOTE: if AUTO-CONFIG is used, the AFES bits in AUTO_CFG_0 register (0x7B) must be set to match

	uint8_t bits;

	// verify input within acceptable range:
	if(set_bits_2 > 3)	// 0 - 3 is allowed
	{
		printf("FFI too big\r\n");
		set_bits_2 = 3;
	}

	pause_MPR121_run_mode();
	// read the register bits:
	bits = touch_read(AFE_CONFIG_1);
	// change the applicable bits:
	bits &=~ 0b11000000;
	bits |= (set_bits_2 << 6);
	// write the revised value:
	touch_write(AFE_CONFIG_1, bits);
	resume_MPR121_run_mode();

	// compute and record the new FFI setting:
	if(set_bits_2 == 0)			first_filter_iterations = 6;
	else if(set_bits_2 == 1)	first_filter_iterations = 10;
	else if(set_bits_2 == 2)	first_filter_iterations = 18;
	else /*if(set_bits == 3)*/	first_filter_iterations = 34;

	return first_filter_iterations;
}

uint8_t set_MPR121_charge_discharge_current_bits(uint8_t set_bits_6)
{	
	// units: [microamperes] (16 is the default, range 1 - 63)
	uint8_t bits;
	
	// verify input within acceptable range:
	if(set_bits_6 == 0)	// 0 is allowed, but not useful
	{	
		printf("electrodes OFF\r\n");
	}
	
	if(set_bits_6 > 63)	// 64 or greater is not permitted
	{	
		printf("current limited to 63 uA\r\n");
		set_bits_6 = 63;
	}

	pause_MPR121_run_mode();
	bits = touch_read(AFE_CONFIG_1);	// read the register bits
	bits &=~ 0b00111111;				// change the applicable bits
	bits |= set_bits_6;
	touch_write(AFE_CONFIG_1, bits);	// write the revised value
	resume_MPR121_run_mode();

	// record the new current setting:
	charge_discharge_current = set_bits_6;	// linear mapping
	
	return charge_discharge_current;
}

float set_MPR121_charge_discharge_time_bits(uint8_t set_bits_3)
{	
	// units: [microseconds] (16 is the default, range 0.5 - 32)	mapping: 0.5*2^(n-1)
	uint8_t bits;
	
	// verify input within acceptable range:
	if(set_bits_3 == 0)	// 0 is invalid
	{	
		printf("CDT too short\r\n");
		set_bits_3 = 1;
	}
	if(set_bits_3 > 7)
	{	
		printf("CDT too long\r\n");
		set_bits_3 = 7;
	}

	pause_MPR121_run_mode();
	bits = touch_read(AFE_CONFIG_2);	// read the register bits
	bits &=~ 0b11100000;				// change the applicable bits
	bits |= (set_bits_3 << 5);
	touch_write(AFE_CONFIG_2, bits);	// write the revised value
	resume_MPR121_run_mode();

	// record the new CDT setting:
	charge_discharge_time = 0.5*(1 << (set_bits_3-1));	// 0.5*2^(n-1)

	return charge_discharge_time;
}

uint8_t set_MPR121_second_filter_iterations_bits(uint8_t set_bits_2)
{	
	// units: [number of samples] (4 is the default, range {4,6,10,18})
	uint8_t bits;
	
	// verify input within acceptable range:
	if(set_bits_2 > 3)	// 0 - 3 is allowed
	{	
		printf("SFI too big\r\n");
		set_bits_2 = 3;
	}

	pause_MPR121_run_mode();
	bits = touch_read(AFE_CONFIG_2);		// read the register bits
	bits &=~ 0b00011000;					// change the applicable bits
	bits |= (set_bits_2 << 3);
	touch_write(AFE_CONFIG_2, bits);		// write the revised value
	resume_MPR121_run_mode();

	// compute and record the new SFI setting:
	if(set_bits_2 == 0)			second_filter_iterations = 4;
	else if(set_bits_2 == 1)	second_filter_iterations = 6;
	else if(set_bits_2 == 2)	second_filter_iterations = 10;
	else /*if(set_bits == 3)*/	second_filter_iterations = 18;

	MPR121_meas_update_interval_ms = (uint16_t)electrode_sampling_interval*second_filter_iterations;

	return second_filter_iterations;
}

uint8_t set_MPR121_electrode_sampling_interval_bits(uint8_t set_bits_3)
{	
	// units: [milliseconds] (16 ms is the default)
	uint8_t bits;
	
	// check that the input bits are within range
	if(set_bits_3 > 7)
	{
		printf("ESI too high");
		set_bits_3 = 7;
	}

	pause_MPR121_run_mode();
	bits = touch_read(AFE_CONFIG_2);		// read the register bits
	bits &=~ 0b00000111;					// change the applicable bits
	bits |= set_bits_3;
	touch_write(AFE_CONFIG_2, bits);		// write the revised value
	resume_MPR121_run_mode();

	// compute and record the sampling interval
	electrode_sampling_interval = 1 << set_bits_3;	// 2^n
	MPR121_meas_update_interval_ms = (uint16_t)electrode_sampling_interval*second_filter_iterations;

	return electrode_sampling_interval;
}

uint8_t enable_MPR121_individual_electrodes_bits(uint8_t set_bits_4)
{
	uint8_t bits;
	
	// check that the input bits are within range
	if(set_bits_4 > 12)
	{
		printf("ELE too high");
		set_bits_4 = 12;
	}
	if(set_bits_4 == 0)
	{
		printf("electrodes disabled!\r\n");
		_delay_ms(4); // print delay
	}

	bits = touch_read(ELE_CFG);		// read the register bits
	bits &=~ 0b00001111;			// change the applicable bits
	bits |= set_bits_4;

	set_MPR121_stop_mode();	// must first put in stop mode before writing a new value
	touch_write(ELE_CFG, bits);		// write the revised value

	// compute and record the individual electrode configuration
	num_individual_electrodes_enabled = set_bits_4;

	return num_individual_electrodes_enabled;
}

uint8_t enable_MPR121_proximity_electrodes_bits(uint8_t set_bits_2)
{
	uint8_t bits;
	
	// check that the input bits are within range
	if(set_bits_2 > 3)
	{
		printf("ELEPROX too high");
		set_bits_2 = 3;
	}
	if(set_bits_2 > 0)
	{
		printf("ELEPROX enabled\r\n");
		_delay_ms(2); // print delay
	}

	bits = touch_read(ELE_CFG);		// read the register bits
	bits &=~ 0b00110000;			// change the applicable bits
	bits |= (set_bits_2 << 4);

	set_MPR121_stop_mode();	// must first put in stop mode before writing a new value
	touch_write(ELE_CFG, bits);		// write the revised value
	
	// compute and record the individual electrode configuration
	if(set_bits_2 == 0)				num_proximity_electrodes_enabled = 0;
	else if(set_bits_2 == 1)		num_proximity_electrodes_enabled = 2;
	else if(set_bits_2 == 2)		num_proximity_electrodes_enabled = 4;
	else /*if(set_bits == 3)*/		num_proximity_electrodes_enabled = 12;

	return num_proximity_electrodes_enabled;
}


/* INTERNAL FUNCTIONS - LOW-LEVEL - RUN MODE & RESET */

void set_MPR121_stop_mode(void)
{
	touch_write(ELE_CFG, 0);
	electrode_configuration_register = 0;
}

void pause_MPR121_run_mode(void)
{
	ecr_pause_value = electrode_configuration_register;
	
	if(electrode_configuration_register != 0)
	{
		touch_write(ELE_CFG, 0);
	}
}

void resume_MPR121_run_mode(void)
{
	if(ecr_pause_value != electrode_configuration_register)
	{
		touch_write(ELE_CFG, ecr_pause_value);

		// acquire new baseline readings
		save_electrode_baseline_values();
	}
}

void touch_soft_reset(void)
{
	// Ref: MPR121 Technical Data sheet, Rev 4, sec 15 (pg. 19)

	// this function calls the init_touch routine, after executing the soft reset command
	
	printf("MPR121 SOFT RESET\r\n");
	_delay_ms(10);

	touch_write(SOFT_RST, 0x63);
	_delay_ms(10);	// unknown if these delays are needed

	init_touch();
	_delay_ms(10);

	touch_existance_confirmed = check_touch_exists();

	if(!touch_existance_confirmed)
		printf("ERROR: lost MPR121\r\n");
}


/* INTERNAL FUNCTIONS - LOW-LEVEL - I2C */

uint8_t touch_read(uint8_t read_address)
{
	uint8_t read_value[1];
	
	I2C_read_bytes(MPR121_address, read_address, read_value, 1);

	return read_value[0];
}

void touch_read_electrodes(uint8_t num_vals, uint16_t* vals)
{
	uint8_t touch_bytes[2*num_vals];
	uint16_t touch_value;
	uint8_t ei;	// electrode index

	I2C_read_bytes(MPR121_address, CAP_ELEC_0, touch_bytes, 2*num_vals);

	for(ei = 0; ei < num_vals; ei++)
	{
		// NOTE: MPR121 uses little endian
		touch_value = touch_bytes[ei*2 + 0];
		touch_value |= touch_bytes[ei*2 + 1] << 8;
		vals[ei] = touch_value;
	}
}

void touch_write(uint8_t addr, uint8_t data)
{
	if(verify_touch_existance())
	{
		switch(addr)
		{
			case AFE_CONFIG_1:
				analog_front_end_register_1 = data;
				AFE_settings_updated = 1;
				break;
			case AFE_CONFIG_2:
				analog_front_end_register_2 = data;
				AFE_settings_updated = 1;
				break;
			case ELE_CFG:
				electrode_configuration_register = data;
				//printf("EC=%u\r\n", data);	// DEBUG
				break;
		}
	
		I2C_write_byte(MPR121_address, addr, data);
	}
}


/* CALIBRATION */

void save_electrode_baseline_values(void)
{
	uint8_t OOR = 0;	// out of range
	uint8_t e_i;		// electrode index

	if(TOUCH_DEBUGMODE >= 2)	printf("new Baseline\r\n");
	
	//get_electrode_values(baseline_C_ADC_vals);
	get_electrodes_values(baseline_C_ADC_vals, MPR121_NUM_ELECTRODES);

	for(e_i = 0; e_i < 8; e_i++)
	{
		if(baseline_C_ADC_vals[e_i] > MPR121_ADC_VMEAS_MAX)
		{
			OOR = 1;
			if(TOUCH_DEBUGMODE)	printf("H%u",e_i);
		}
	}

	if(OOR)
	{
		if(TOUCH_DEBUGMODE)	printf("\r\n");
	}
}


void sweep_AFE_calibration(void)
{
	uint8_t Ibits;	// acceptable range: 1 - 0b111111
	uint8_t I;
	float T;
	float Q;
	uint8_t e_i;	// electrode index

	uint16_t ADC_vals[MPR121_NUM_ELECTRODES];

	request_overtime_permission();

	//for(Ibits = 1; Ibits <= 63; Ibits++)
	//for(Ibits = 1; Ibits <= 3; Ibits++)
	for(Ibits = 1; Ibits <= 63; Ibits++)
	{
		set_MPR121_charge_discharge_current_bits(Ibits);
	
		_delay_ms(5);

		I = get_MPR121_CDC_uA();
		T = get_MPR121_CDT_us();
		Q = get_MPR121_applied_charge_pC();
		
		//get_electrode_values(ADC_vals);
		get_electrodes_values(ADC_vals, MPR121_NUM_ELECTRODES);

		printf("%u", I);
		printf(",%.1f", T);
		printf(",%.1f", Q);

		for(e_i = 0; e_i < MPR121_NUM_ELECTRODES; e_i++)
			printf(",%u", ADC_vals[e_i]);
	
		printf("\r\n");

		_delay_ms(15);	// for data collection only, long delay
	}

	//set_MPR121_charge_discharge_current_bits(1);	// leave in a low current state (maybe caused bug?)
}

/* INITIALIZATION */

void init_MPR121_registers(void)
{
	set_MPR121_stop_mode();	// do this in case of XMEGA reset without power cycling (e.g. reprogramming)
	
	/* Set the baseline control filtering */
	
	// filtering when data is > baseline
	touch_write(MHD_R, 0x01);
	touch_write(NHD_R, 0x01);
	touch_write(NCL_R, 0x00);
	touch_write(FDL_R, 0x00);
	// filtering when data is < baseline
	touch_write(MHD_F, 0x01);
	touch_write(NHD_F, 0x01);
	touch_write(NCL_F, 0xFF);
	touch_write(FDL_F, 0x02);

	/* Set touch and release thresholds for each electrode */

	// note: these are ridiculously low for our application
	uint8_t TOU_THRESH = 15;
	uint8_t REL_THRESH = 10;
	touch_write(ELE0_T, TOU_THRESH);
	touch_write(ELE0_R, REL_THRESH);
	touch_write(ELE1_T, TOU_THRESH);
	touch_write(ELE1_R, REL_THRESH);
	touch_write(ELE2_T, TOU_THRESH);
	touch_write(ELE2_R, REL_THRESH);
	touch_write(ELE3_T, TOU_THRESH);
	touch_write(ELE3_R, REL_THRESH);
	touch_write(ELE4_T, TOU_THRESH);
	touch_write(ELE4_R, REL_THRESH);
	touch_write(ELE5_T, TOU_THRESH);
	touch_write(ELE5_R, REL_THRESH);

	/* Set the Analog Front End configuration */
	
	// AFE Configuration Register 1:
	set_MPR121_first_filter_iterations_bits(0);		// 0 = 6 iterations (Default)
	set_MPR121_charge_discharge_current_bits(CDC_DEFAULT_BITS);	// 16 = 16 uA (Default)
	
	// AFE Configuration Register 2:
	set_MPR121_charge_discharge_time_bits(CDT_DEFAULT_BITS);	// 1 = 0.5 us (Default)
	set_MPR121_second_filter_iterations_bits(0);				// 0 = 4 samples (Default)
	set_MPR121_electrode_sampling_interval_bits(4);				// 4 = 16 ms (Default)

	/* Set the electrode configuration */
	
	//enable_MPR121_individual_electrodes_bits(0);	// no individual electrodes (Default)
	//enable_MPR121_individual_electrodes_bits(6);	// individual detection on electrodes 0-5
	//enable_MPR121_individual_electrodes_bits(8);	// individual detection on electrodes 0-8
	enable_MPR121_individual_electrodes_bits(12);	// individual detection on electrodes 0-11
	
	enable_MPR121_proximity_electrodes_bits(0); 	// no proximity detection (Default)
	//enable_MPR121_proximity_electrodes_bits(1);	// proximity detection on electrodes 0-1	
	//enable_MPR121_proximity_electrodes_bits(2);	// proximity detection on electrodes 0-3	
	//enable_MPR121_proximity_electrodes_bits(3);	// proximity detection on electrodes 0-11
	
	/* Enable Auto Config and auto Reconfig	*/
	/*
	touch_write(AUTO_CFG_0, 0x0B); // 0x0B is Default
	// 0b00001011 = 0x0B
	// 0b00------ = FFI = 6 (note: this MUST match the FFI set in the AFE for AUTO-CONFIG to function correctly)
	// 0b--00---- = Retry Disabled
	// 0b----10-- = Baseline Adjust Value, baseline set to AUTO-CONFIG baseline with lower 3 bits cleared (?)
	// 0b------1- = Automatic Reconfiguration Enable, enabled
	// 0b-------1 = Automatic Configuration Enable, enabled
	
	// Vdd = 3.3V
	touch_write(AUTO_CFG_USL, 202);	// USL = (Vdd-0.7)/Vdd*256 = 201.696 ~= 202
	touch_write(AUTO_CFG_LSL, 131);	// LSL = 0.65*USL = 131.10 ~= 131 
	touch_write(AUTO_CFG_TL, 182);	// Target = 0.9*USL = 181.8 ~= 182
	*/
}

uint8_t init_touch(void)
{
	touch_existance_confirmed = check_touch_exists();
	
	if(touch_existance_confirmed)
	{
		init_MPR121_registers();

		_delay_ms(50);

		save_electrode_baseline_values();
	}
	
	return touch_existance_confirmed;
}