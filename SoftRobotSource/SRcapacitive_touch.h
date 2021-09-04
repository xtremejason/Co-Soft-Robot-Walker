//SRcapacitive_touch.h

#include "SoftRobotI2C.h"

#ifdef __cplusplus
extern "C"
{
#endif


// see Table 1 of application note AN3895 for these values (ADDR pin is pin 4 of the 20-pin QFN package)
#define MPR121_I2C_ADDR_PIN_ON_GND		0x5A
#define MPR121_I2C_ADDR_PIN_ON_VCC		0x5B
#define MPR121_I2C_ADDR_PIN_ON_SDA		0x5C
#define MPR121_I2C_ADDR_PIN_ON_SCL		0x5D

//#define MPR121_ADDRESS		MPR121_I2C_ADDR_PIN_ON_GND	// this is a standard 7-BIT address!
#define MPR121_VDD						3.3	// volts, supplied voltage
#define MPR121_DATA_EXPIRATION_TIME_ms	40	// should be the time it takes for the main loop to complete

#define MPR121_NUM_ELECTRODES			12 // indexed 0 to 11


/* USER ACCESSIBLE FUNCTIONS */

uint8_t get_electrodes_values(uint16_t *cap_values, uint8_t num_electrodes);
// returns boolean: 1 if new values are collected, 0 if old values are recycled
// special note: new baseline values are SAVED when cap_values* points to baseline_C_ADC_vals[]

void get_electrodes_baseline(uint16_t *cap_values, uint8_t num_electrodes);
// this only reports the baseline values, it does not cause an update of the baseline values,
//	to save a new baseline array, use the get_electrodes_values() function

/* USER ACCESSIBLE FUNCTIONS - CONVERSIONS */

float C_ADC_to_capacitance(uint16_t C_ADC);

float C_ADC_to_voltage(uint16_t C_ADC);


/* USER ACCESSIBLE FUNCTIONS - MPR121 SETTINGS */

uint8_t get_MPR121_CDC_uA(void);

float get_MPR121_CDT_us(void);

float get_MPR121_applied_charge_pC(void);


/* DEBUG FUNCTIONS - PRINT ONLY! */

void print_electrode_values(void);

void MPR121_debug_printout1(void);

void MPR121_debug_printout2(void);

void MPR121_debug_printout3(void);


/* INTERNAL FUNCTIONS - HIGH LEVEL */

// get MPR121 register value associated with the given touch pad number
uint8_t electrode_register_addr(uint8_t electrode_index);
// input: electrode index (range is 0 - 11)
// output: register address (byte) in the MPR121 memory bank

// only need to perform this once (during init)
uint8_t check_touch_exists(void);

// call this to get check if touch sensor was found on the i2C bus
// prints an ERROR MESSAGE if user tried to access an absent touch sensor
uint8_t verify_touch_existance(void);
// returns boolean: 1 if MPR121 exists, 0 otherwise

// boolean check, typically called by functions that return capacitance values
// used to determine if the most recent values are still valid to report them
uint8_t touch_data_is_fresh(void);


/* INTERNAL FUNCTIONS - LOW-LEVEL - ANALOG FRONT END SETTINGS */

uint8_t set_MPR121_first_filter_iterations_bits(uint8_t set_bits_2);

// charge-discharge current [microamperes] (16 is the default, range 1 - 63)
uint8_t set_MPR121_charge_discharge_current_bits(uint8_t set_bits_6);

// charge-discharge time [microseconds] (16 is the default, range 0.5 - 32, 0.5*2^(n-1))
float set_MPR121_charge_discharge_time_bits(uint8_t set_bits_3);

uint8_t set_MPR121_second_filter_iterations_bits(uint8_t set_bits_2);

uint8_t set_MPR121_electrode_sampling_interval_bits(uint8_t set_bits_3);

uint8_t enable_MPR121_individual_electrodes_bits(uint8_t set_bits_4);

uint8_t enable_MPR121_proximity_electrodes_bits(uint8_t set_bits_2);


/* INTERNAL FUNCTIONS - LOW-LEVEL - RUN MODE & RESET */

// disable all electrodes
void set_MPR121_stop_mode(void);

// temporarily disable all electrodes
void pause_MPR121_run_mode(void);

// (re)enable previously running electrodes
void resume_MPR121_run_mode(void);

// resets the MPR121 without power cycling the device
void touch_soft_reset(void);


/* INTERNAL FUNCTIONS - LOW-LEVEL - I2C */

// read 8bit data from a single MPR121 register via I2C
uint8_t touch_read(uint8_t read_address);

// read via I2C MPR121 conversion result (ADC-10 bit) data from electrodes 0 to (num_vals - 1)
// vals is an array to fill of length num_vals
void touch_read_electrodes(uint8_t num_vals, uint16_t* vals);

// write 8bit data to a single MPR121 register via I2C
void touch_write(uint8_t register_address, uint8_t data);


/* CALIBRATION */

// take a baseline touch reading from the sensor strip and save these values, calibration
void save_electrode_baseline_values(void);

void sweep_AFE_calibration(void);

/* INITIALIZATION */

// set up MPR121 for our selected default settings
void init_MPR121_registers(void);

// initialize touch controller (return 1 if MPR121 was found on I2C bus)
uint8_t init_touch(void);


#ifdef __cplusplus
}
#endif
