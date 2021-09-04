// SRxmega_ADC.h

#ifndef __SR_XMEGA_ADC_H__
#define __SR_XMEGA_ADC_H__

// Currently, this ADC driver library may be specific to the Soft Robot project.
// ADCs have been used in all of the projects so far, including:
// Droplets (IR sensor values)
// Hex Blocks (high-voltage supply measurement)
// SoundBoard (microphone)
// Soft Robot (pressure sensors and strain sensors)
// Plan is to make this library generic to ADC routines (agnostic of project)

//#include <avr/io.h>		// provides uint8_t
#include <avr/pgmspace.h>	// provides uint8_t, PORT_t, ADC_t


#ifdef __cplusplus
extern "C"
{
#endif



// Initialize the specified pin for ADC use
// Vref_choice:
//	0: Vcc/1.6 (normal operation)
//	1: Vcc/2
//	2: Analog Reference A (externally supplied)
//	3: Analog Reference B (externally supplied)
void ADC_init(ADC_t *sensor_adc, PORT_t *sensor_port, uint8_t sensor_PIN_bm, uint8_t Vref_choice);


// Outputs GND signal on the specified pin, then measures its voltage, then returns the pin to normal ADC input mode  
uint8_t get_ADC_zero(ADC_t *sensor_adc, uint8_t sensor_MUX_gc, PORT_t *sensor_port, uint8_t sensor_pin_bm);


#ifdef __cplusplus
}
#endif

#endif
