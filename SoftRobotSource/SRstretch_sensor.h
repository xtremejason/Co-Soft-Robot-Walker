// SRstretch_sensor.h

#ifndef __SRSTRETCH_SENSOR_H__
#define __SRSTRETCH_SENSOR_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif

void SS_init(void);

// sensor_num = {1, ... 6}
uint8_t get_ADC_stretch(uint8_t sensor_num);

void get_SS_zeros(uint8_t* zeros);

float get_strain_curvature(uint8_t sensor_num);

#ifdef __cplusplus
}
#endif

#endif
