
#ifndef __MAIN_H
#define __MAIN_H


#include <math.h>
#include <avr/io.h>
//#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
//#include <stdlib.h> // used for rand()


#include "SOFTROBOTboard.h"	
//#include "usart.h" // despite the name appearance, this was proprietary to xgrid
#include "xgrid_usart.h"  // the "new" usart.h

#include "eeprom.h"
#include "xgrid.h"

#include "../xboot/xbootapi.h"

#include "../SoftRobotSource/SoftRobotI2C.h"

#include "../SoftRobotSource/SRstatus_LED.h"
#include "../SoftRobotSource/SRpump.h"
#include "../SoftRobotSource/SRvalves.h"
#include "../SoftRobotSource/SRburners.h"
#include "../SoftRobotSource/SRstretch_sensor.h"
#include "../SoftRobotSource/SRanalog_pressure.h"
#include "../SoftRobotSource/SRdigital_pressure.h"
#include "../SoftRobotSource/SRPC_com.h"
#include "../SoftRobotSource/software_clock.h"
#include "../SoftRobotSource/SRaccelerometer.h"
#include "../SoftRobotSource/SRdigital_barometer.h"
#include "../SoftRobotSource/SRcapacitive_touch.h"
#include "../SoftRobotSource/SRbuttons.h"
#include "../SoftRobotSource/SRexternal_device.h"
#include "../SoftRobotSource/SRmisc_functions.h"
#include "../SoftRobotSource/SRhardware_calibration.h"
#include "../SoftRobotSource/SR9axis_IMU.h"

#include "../SoftRobotControl/SRpressure_control.h"
#include "../SoftRobotControl/SRdata_record.h"
//#include "../SoftRobotControl/SRreservoir_control.h"
#include "../SoftRobotControl/SRmotion_control.h"
#include "../SoftRobotControl/SRtouch_control.h"


// Prototypes
void init(void);
int main(void);
uint8_t SP_ReadCalibrationByte( uint8_t index );
uint8_t SP_ReadUserSigRow( uint8_t index );

#ifndef ADCACAL0_offset

#define ADCACAL0_offset 0x20
#define ADCACAL1_offset 0x21
#define ADCBCAL0_offset 0x24
#define ADCBCAL1_offset 0x25

#endif

#endif // __MAIN_H



