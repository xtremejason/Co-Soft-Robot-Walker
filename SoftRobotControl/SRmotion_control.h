//SRmotion_control.h

#ifndef __SR_MOTION_CONT_H__
#define __SR_MOTION_CONT_H__


//#include <avr/io.h>
//#include <avr/interrupt.h>
#include <stdio.h> // printf

//#include "../SoftRobotSource/SRanalog_pressure.h"
//#include "../SoftRobotSource/SRdigital_pressure.h"
//#include "../SoftRobotSource/SRvalves.h"
//#include "../SoftRobotSource/SRpump.h"
//#include "../SoftRobotSource/SRmisc_functions.h"


#define GRASP_TIMER				1

#define GRASP_ACT_1				3
#define GRASP_ACT_2				1

#define WALK_ACT_1				1
#define WALK_ACT_2				3
#define WALK_ACT_3				4
#define WALK_ACT_4				6


#define NO_MOTION				0
#define MOTION_STAND_UP 		10
#define YOUNG_MOTION			11
#define MOTION_STEP				1
#define MOTION_STEP2			2

#define GRASP_PRESSURE_psi		6
#define GRASP_RELAX_psi			2

#define STAND_PRESSURE_psi		8



#ifdef __cplusplus
extern "C"
{
#endif


/* USER ACCESSIBLE FUNCTIONS */

uint8_t motion_in_progress(void);

void begin_motion(uint8_t motion_description);

uint8_t motion_subgoal_reached(void);

// call this when the actuator has reached its desired goal, for when coordinated motions in progress
void clear_actuator_motion_checkpoint(uint8_t act_num);

/* INTERNAL FUNCTIONS */

void motion_handler(void);

/* INITIALIZATION */

void begin_submotion(uint8_t motion_stage);


#ifdef __cplusplus
}
#endif

#endif