// SRhardware_calibration.h

#ifndef __SRHARDWARE_CALIBRATION_H__
#define __SRHARDWARE_CALIBRATION_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif

#define CALIBRATION_DEBUG_PRINT 		0
//#define CALIBRATION_DEBUG_PRINT 		1



uint8_t hardware_calibration_main_PCB_ID(void);

uint8_t hardware_calibration_actuator_PCB_ID(uint8_t act_num);

void hardware_calibration_ADCtoPSI_values(uint8_t actuator_PCB_ID, float *slope, float *yint);

void set_pump_ADC_drag_coef(void);
// needs called as part of init somewhere, currently in load_default_ADCtoPSI_calibration()

float pressure_drag_drop_correction(float p_meas, uint8_t act_num);

void load_default_ADCtoPSI_calibration(void);

void clear_all_ADCtoPSI_calibration(void);



#ifdef __cplusplus
}
#endif

#endif