// SRstretch_sensor.c

#include "SRstretch_sensor.h"

#include <util/delay.h>
#include <stdio.h>

#include "SRmisc_functions.h"	// for find_median_of_3()


#define STRETCH1_PORT 		PORTA
#define STRETCH1_PIN_bm		PIN2_bm
#define STRETCH2_PORT 		PORTB
#define STRETCH2_PIN_bm		PIN3_bm
#define STRETCH3_PORT 		PORTB
#define STRETCH3_PIN_bm		PIN0_bm
#define STRETCH4_PORT 		PORTA
#define STRETCH4_PIN_bm		PIN7_bm
#define STRETCH5_PORT 		PORTA
#define STRETCH5_PIN_bm		PIN1_bm
#define STRETCH6_PORT 		PORTA
#define STRETCH6_PIN_bm		PIN5_bm


#define SS1_ADC		ADCA		// SS 1 on PA2
#define SS2_ADC		ADCB		// SS 2 on PB3
#define SS3_ADC		ADCB		// SS 3 on PB0
#define SS4_ADC		ADCA		// SS 4 on PA7
#define SS5_ADC		ADCA		// SS 5 on PA1
#define SS6_ADC		ADCA		// SS 6 on PA5


#define SS1_MUXPOS_gc		ADC_CH_MUXPOS_PIN2_gc		// SS 1 on PA2
#define SS2_MUXPOS_gc		ADC_CH_MUXPOS_PIN3_gc		// SS 2 on PB3
#define SS3_MUXPOS_gc		ADC_CH_MUXPOS_PIN0_gc		// SS 3 on PB0
#define SS4_MUXPOS_gc		ADC_CH_MUXPOS_PIN7_gc		// SS 4 on PA7
#define SS5_MUXPOS_gc		ADC_CH_MUXPOS_PIN1_gc		// SS 5 on PA1
#define SS6_MUXPOS_gc		ADC_CH_MUXPOS_PIN5_gc		// SS 6 on PA5


void SS_init()
{
	// Stretch Sensors use ADCA/ADCB channel 1, all the time

	/* (1) PORT CONFIGURATION */
	// SET INPUT PINS AS INPUTS 
	
	STRETCH1_PORT.DIRCLR = STRETCH1_PIN_bm;
	STRETCH2_PORT.DIRCLR = STRETCH2_PIN_bm;
	STRETCH3_PORT.DIRCLR = STRETCH3_PIN_bm;
	STRETCH4_PORT.DIRCLR = STRETCH4_PIN_bm;
	STRETCH5_PORT.DIRCLR = STRETCH5_PIN_bm;
	STRETCH6_PORT.DIRCLR = STRETCH6_PIN_bm;

	/* (2) ADC CONFIGURATION */

	// 28.16.3 REFCTRL – Reference Control register
	//
	// Bit 1 – BANDGAP: Bandgap enable
	// Setting this bit enables the Bandgap for ADC measurement. Note that if any other functions are
	// using the Bandgap already, this bit does not need to be set when the internal 1.00V reference is
	// used in ADC or DAC, or if the Brown-out Detector is enabled.
	//
	// Bits 6:4 – REFSEL[2:0]: ADC Reference Selection
	// These bits selects the reference for the ADC

	SS1_ADC.REFCTRL = ADC_REFSEL_VCC_gc;							// Vcc/1.6
	SS2_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	SS3_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	SS4_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	SS5_ADC.REFCTRL = ADC_REFSEL_VCC_gc;
	SS6_ADC.REFCTRL = ADC_REFSEL_VCC_gc;

	//SS3_ADC.REFCTRL = 0b01000000;								// Vcc/2
	//ADCB.REFCTRL = 0b00100000;									// AREFA = not provided
	//ADCB.REFCTRL = 0b00110000;									// AREFB = not provided

	// 28.16.2 CTRLB – ADC Control Register B
	//
	// Bit 7 – IMPMODE: Gain Stage Impedance Mode
	// This bit controls the impedance mode of the gain stage. See GAIN setting with ADC Channel
	// Register description for more information.
	//
	// Bit 6:5 – CURRLIMIT[1:0]: Current Limitation
	// These bits can be used to limit the maximum current consumption of the ADC. Setting these bits
	// will also reduce the maximum sampling rate. The available settings is shown in Table 28-3 on
	// page 367. The indicated current limitations are nominal values, refer to device datasheet for
	// actual current limitation for each setting.
	//
	// Bit 4 – CONVMODE: ADC Conversion Mode
	// This bit controls whether the ADC will work in signed or unsigned mode. By default this bit is
	// cleared and the ADC is configured for unsigned mode. When this bit is set the ADC is configured
	// for signed mode.
	//
	// Bit 3 – FREERUN: ADC Free Running Mode
	// When the bit is set to one, the ADC is in free running mode and ADC channels defined in the
	// EVCTRL register are swept repeatedly.
	//
	// Bit 2:1 – RESOLUTION[1:0]: ADC Conversion Result Resolution
	// These bits define whether the ADC completes the conversion at 12- or 8-bit result. They also
	// define whether the 12-bit result is left or right oriented in the 16-bit result registers. See Table
	// 28-4 on page 367 for possible settings.
	
	SS1_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;		// use 8 bit resolution
	SS2_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	SS3_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	SS4_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	SS5_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
	SS6_ADC.CTRLB = ADC_RESOLUTION_8BIT_gc;
			
			/* TROUBLES: currently, we cannot use the 12-bit resolution setting because we cannot
			read the TEMP register of the ADC, because the compiler does not recognize the TEMP
			register, because iox128a3u.h is NOT being used, rather iox128a3.h is being used.
			It is our makefile that governs this choice, because the makefile also is used for
			programming "make program" (links with avrdude) and avrdude does not support A3U,
			moreover WINAVR 'avr\include\avr\' folder's io.h does not recognize __AVR_ATxmega128A3U__
			
			??? I don't know where in the compile pipeline the makefile line:

			MCU = atxmega128a3u
			
						gets converted to the command in <io.h>:

			#elif defined (__AVR_ATxmega128A3U__)
			#  include <avr/iox128a3u.h>


			see: www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=120482&start for possibly
			helpful discussion
			??? ------------------------------------------------------------------------------ */

	// 28.16.5 PRESCALER – Clock Prescaler Register
	//
	// Bit 2:0 – PRESCALER[2:0]: ADC Prescaler configuration
	// These bits define the ADC clock relative to the Peripheral clock, according to Table 28-9 on
	// page 370.

	SS1_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	SS2_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	SS3_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	SS4_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	SS5_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;
	SS6_ADC.PRESCALER = ADC_PRESCALER_DIV512_gc;

	/* (3) ADC CHANNEL CONFIGURATION */
	
	SS1_ADC.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	SS2_ADC.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	SS3_ADC.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	SS4_ADC.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	SS5_ADC.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	SS6_ADC.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;

	/* (4) READ & LOAD CALIBRATION (PRODUCTION SIGNATURE ROW) */	//TODO (unlikely will make any difference)
	
	/*
	ADCBcalibration0 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCBCAL0 ));
	ADCBcalibration1 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCBCAL1 ));
	ADCAcalibration0 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCACAL0 ));
	ADCAcalibration1 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCACAL1 ));
	*/

	// SHOULD WE WRITE THEM???
	
	//ADCB.CALL = ADCBcalibration0;
	//ADCB.CALH = ADCBcalibration1;
	
	/* (5) ENABLE THE ADC SO THAT IT MAY MAKE MEASUREMENTS */

	SS1_ADC.CTRLA = ADC_ENABLE_bm;
	SS2_ADC.CTRLA = ADC_ENABLE_bm;
	SS3_ADC.CTRLA = ADC_ENABLE_bm;
	SS4_ADC.CTRLA = ADC_ENABLE_bm;
	SS5_ADC.CTRLA = ADC_ENABLE_bm;
	SS6_ADC.CTRLA = ADC_ENABLE_bm;

	/* (6) FIND AND RECORD THE ZERO-OFFSET OF EACH IR DIRECTION (optional) */

	//get_SS_zeros(ADC_zero_reading);

}


uint8_t get_ADC_stretch(uint8_t sensor_num)
{
	// this routine uses ADCA/ADCB channel 1, all the time
	
	uint8_t meas[3];
	//uint16_t average;
	uint16_t median;
	//uint8_t scaled_median, scaled_average;
	
	ADC_t *sensor_adc;
	uint8_t sensor_MUX_gc;

	switch(sensor_num)
	{
		case 1:
			sensor_adc = &SS1_ADC;
			sensor_MUX_gc = SS1_MUXPOS_gc;
			break;
		case 2:
			sensor_adc = &SS2_ADC;
			sensor_MUX_gc = SS2_MUXPOS_gc;
			break;
		case 3:
			sensor_adc = &SS3_ADC;
			sensor_MUX_gc = SS3_MUXPOS_gc;
			break;
		case 4:
			sensor_adc = &SS4_ADC;
			sensor_MUX_gc = SS4_MUXPOS_gc;
			break;
		case 5:
			sensor_adc = &SS5_ADC;
			sensor_MUX_gc = SS5_MUXPOS_gc;
			break;
		case 6:
			sensor_adc = &SS6_ADC;
			sensor_MUX_gc = SS6_MUXPOS_gc;
			break;
		default:
			return 0;
	}
	
	// (re)connect the MUX:
	(*sensor_adc).CH1.MUXCTRL &= ~0b01111000;	// clear out the old MUXPOS_gc
	(*sensor_adc).CH1.MUXCTRL |= sensor_MUX_gc;

	for (uint8_t i = 0; i < 3; i++)
	{
		(*sensor_adc).CTRLA |= ADC_CH1START_bm;
		while ((*sensor_adc).CH1.INTFLAGS==0){};		// wait for 'complete flag' to be set
		(*sensor_adc).CH1.INTFLAGS = 1;					// clear the complete flag 
		meas[i] = (*sensor_adc).CH1.RES;
	}		

	//average = ((uint16_t)meas[0] + (uint16_t)meas[1] + (uint16_t)meas[2]) / 3;
	median = find_median_of_3(meas);

	// ADC_zero_reading is what is returned for any voltage vale that is below the "activation voltage"
	// so a simple subtraction is probably not the correct way to handle this (below code is wrong)
	//scaled_average = (uint8_t)average - ADC_zero_reading[sensor_num-1];
	//scaled_median = find_median_of_3(meas) - ADC_zero_reading[sensor_num-1];
	
	//printf("meas3: %i %i %i\r\n",meas[0], meas[1], meas[2]);
	//printf("uavg: %i\r\n",scaled_average);
	//printf("umed: %i\r\n",scaled_median);

	//return scaled_average;
	//return scaled_median;
	return median;
}


void get_SS_zeros(uint8_t* zeros)
{
	ADC_t *sensor_adc;
	uint8_t sensor_MUX_gc;
	PORT_t *sensor_port;
	uint8_t sensor_pin_bm;

	for(uint8_t sensor_num = 1; sensor_num <= 6; sensor_num++)
	{
		switch(sensor_num)
		{
			case 1:
				sensor_adc = &SS1_ADC;
				sensor_MUX_gc = SS1_MUXPOS_gc;
				sensor_port = &STRETCH1_PORT;
				sensor_pin_bm = STRETCH1_PIN_bm;
				break;
			case 2:
				sensor_adc = &SS2_ADC;
				sensor_MUX_gc = SS2_MUXPOS_gc;
				sensor_port = &STRETCH2_PORT;
				sensor_pin_bm = STRETCH2_PIN_bm;
				break;
			case 3:
				sensor_adc = &SS3_ADC;
				sensor_MUX_gc = SS3_MUXPOS_gc;
				sensor_port = &STRETCH3_PORT;
				sensor_pin_bm = STRETCH3_PIN_bm;
				break;
			case 4:
				sensor_adc = &SS4_ADC;
				sensor_MUX_gc = SS4_MUXPOS_gc;
				sensor_port = &STRETCH4_PORT;
				sensor_pin_bm = STRETCH4_PIN_bm;
				break;
			case 5:
				sensor_adc = &SS5_ADC;
				sensor_MUX_gc = SS5_MUXPOS_gc;
				sensor_port = &STRETCH5_PORT;
				sensor_pin_bm = STRETCH5_PIN_bm;
				break;
			case 6:
				sensor_adc = &SS6_ADC;
				sensor_MUX_gc = SS6_MUXPOS_gc;
				sensor_port = &STRETCH6_PORT;
				sensor_pin_bm = STRETCH6_PIN_bm;
				break;
		}
		
		(*sensor_port).DIRSET = sensor_pin_bm;		// set the sense pin as OUTPUT
		(*sensor_port).OUTCLR = sensor_pin_bm;		// put a low voltage on this pin (typically, this will be about 15 mV)

		// (re)connect the MUX:
		(*sensor_adc).CH1.MUXCTRL &= ~0b01111000;	// clear out the old MUXPOS_gc
		(*sensor_adc).CH1.MUXCTRL |= sensor_MUX_gc;
	
		// get the measurement:
		(*sensor_adc).CTRLA |= ADC_CH1START_bm;
		while ((*sensor_adc).CH1.INTFLAGS==0){};	// wait for 'complete flag' to be set
		(*sensor_adc).CH1.INTFLAGS = 1;				// clear the complete flag
		zeros[sensor_num-1] = (*sensor_adc).CH1.RES;
	
		(*sensor_port).DIRCLR = sensor_pin_bm;		// set the sense pin back as INPUT
	}

	
	//printf("ADC offsets\r\n");
	printf("1: %i\r\n",zeros[0]);
	printf("2: %i\r\n",zeros[1]);
	_delay_ms(10);	
	printf("3: %i\r\n",zeros[2]);
	printf("4: %i\r\n",zeros[3]);
	printf("5: %i\r\n",zeros[4]);
	printf("6: %i\r\n",zeros[5]);
	_delay_ms(10);
	
}

#define NOMINAL_SENSOR_R_ADC 77
#define NOMINAL_LEADS_R_ADC	 21

float get_strain_curvature(uint8_t sensor_num)
{
	float curvature;
	
	float actuator_height = 0.02;
	
	float delta_R = get_ADC_stretch(sensor_num) - NOMINAL_SENSOR_R_ADC;
	float R = NOMINAL_SENSOR_R_ADC - NOMINAL_LEADS_R_ADC;
	
	if(delta_R < 0)
		delta_R = 0;
	
	curvature = (sqrt(1.0 + delta_R/R) - 1.0)/actuator_height;
	
	return curvature;
}