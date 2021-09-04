// SRxmega_ADC.c

#include <util/delay.h>		// used in get_ADC_zero()
#include <stdio.h>

#include "SRxmega_ADC.h"

//#include <avr/pgmspace.h>	// provides PORT_t, ADC_t


/*	IF READING CALIBRATION BYTES TO SET THE ADC CALIBRATION (unknown if this is supported)		
#include "stddef.h"				// required to use "offsetof(_,_)" for production row access
#include "../SoftRobotMain/xgrid_maintenance.h"	// required to use "SP_ReadCalibrationByte"
*/


/*
uint8_t ADC_zeros[6];			// this value will typically be about 11 if using Vcc/1.6 reference (unsigned)
								// this is effectively the 'DELTA'V that is used in the math of the ADC in the manual
*/


void ADC_init(ADC_t *sensor_adc, PORT_t *sensor_port, uint8_t sensor_PIN_bm, uint8_t Vref_choice)
{

	/* (1) PORT CONFIGURATION */
	// SET INPUT PINS AS INPUTS 
	
	(*sensor_port).DIRCLR = sensor_PIN_bm;
	
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

	switch(Vref_choice)
	{
		// TODO: keep track of what the setup config is so that readings can be converted back to voltages

		case 0:	// STANDARD
			(*sensor_adc).REFCTRL = ADC_REFSEL_VCC_gc;			// Vcc/1.6
			break;
		case 1: 
			(*sensor_adc).REFCTRL = 0b01000000;					// Vcc/2
			break;
		case 2:	// external supplied reference voltage (AREFA)
			(*sensor_adc).REFCTRL = 0b00100000;						// AREFA = not provided (TODO: what pin(s) would it be on?)
			break;
		case 3: // external supplied reference voltage (AREFB)
			(*sensor_adc).REFCTRL = 0b00110000;						// AREFB = not provided (TODO: what pin(s) would it be on?)
			break;
		default:
			printf("ERROR invalid Vref: %u\r\n", Vref_choice);
			// TODO: halt program?
			break;
	}

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
	
	(*sensor_adc).CTRLB = ADC_RESOLUTION_8BIT_gc;		// use 8 bit resolution
			
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

	(*sensor_adc).PRESCALER = ADC_PRESCALER_DIV512_gc;
	
	/* (3) ADC CHANNEL CONFIGURATION */
	
	(*sensor_adc).CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	
	/* (4) READ & LOAD CALIBRATION (PRODUCTION SIGNATURE ROW) */
	
	/*
	ADCBcalibration0 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCBCAL0 ));
	ADCBcalibration1 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCBCAL1 ));
	ADCAcalibration0 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCACAL0 ));
	ADCAcalibration1 = SP_ReadCalibrationByte(offsetof( NVM_PROD_SIGNATURES_t, ADCACAL1 ));
	*/

	// TODO: SHOULD WE WRITE THEM???  (unlikely will make any difference)
	
	//(*sensor_adc).CALL = ADCBcalibration0;
	//(*sensor_adc).CALH = ADCBcalibration1;
	
	/* (5) ENABLE THE ADC SO THAT IT MAY MAKE MEASUREMENTS */

	(*sensor_adc).CTRLA = ADC_ENABLE_bm;
	
	/* (6) FIND AND RECORD THE ZERO-OFFSET OF EACH IR DIRECTION (optional) */

	// TODO: sensor_MUX_gc needs to be passed to this function to implement
	//ADC_zero_reading = get_ADC_zero(*sensor_adc, sensor_MUX_gc, *sensor_port, sensor_pin_bm);

}




uint8_t get_ADC_zero(ADC_t *sensor_adc, uint8_t sensor_MUX_gc, PORT_t *sensor_port, uint8_t sensor_pin_bm)
{
	// It is hard-coded here that only ADC channel 1 is being used!
	// TODO: is that a problem? (no magic numbers!) 
	
	uint8_t result;

		
	(*sensor_port).DIRSET = sensor_pin_bm;		// set the sense pin as OUTPUT
	(*sensor_port).OUTCLR = sensor_pin_bm;		// put a low voltage on this pin (typically, this will be about 15 mV)

	// (re)connect the MUX:
	(*sensor_adc).CH1.MUXCTRL &= ~0b01111000;	// clear out the old MUXPOS_gc
	(*sensor_adc).CH1.MUXCTRL |= sensor_MUX_gc;
	
	// get the measurement:
	(*sensor_adc).CTRLA |= ADC_CH1START_bm;
	while ((*sensor_adc).CH1.INTFLAGS==0){};	// wait for 'complete flag' to be set
	(*sensor_adc).CH1.INTFLAGS = 1;				// clear the complete flag
	result = (*sensor_adc).CH1.RES;
	
	(*sensor_port).DIRCLR = sensor_pin_bm;		// set the sense pin back as INPUT

	printf("ADC zero (offset): %u\r\n", result);
	_delay_ms(10);	
	
	return result;
	
	// TODO: save the zero result
}
