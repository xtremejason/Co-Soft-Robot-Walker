//SRmotion_control.c

#include "SRmotion_control.h"

#include "../SoftRobotSource/software_clock.h"	// for set_misc_countdown_timer()
#include "SRpressure_control.h"					// for actuator_set_and_forget()
#include "SRdata_record.h"						// for record_data()

#define MOTION_CONTROL_DEBUG    	0	// no printing
//#define MOTION_CONTROL_DEBUG    	1	// print sub-motion stages
//#define MOTION_CONTROL_DEBUG  	2	// print debug symbols
//#define MOTION_CONTROL_DEBUG  	3	// print load motion sequence


#define GRASP_PHASE_LENGTH_ms	2000

#define RECORD_MOTION			0

// TODO: add the ability to 'hold' a pressure for a set amount of time

// CHECKPOINT SET PRESSURES:
#define M0	0
#define S1	3
#define S2	4.5
#define S3	5.0

float stand[4][8] = {	{S1,S1,S1,S1,S2,S2,S2,S2},
						{M0,S1,S1,S1,S1,S2,S2,S2},
						{M0,M0,S1,S1,S1,S1,S2,S2},
						{M0,M0,M0,S1,S1,S1,S1,S2}};
uint8_t stand_len = 8;


float step[4][8] = {	{M0,M0,S2,S2,S2,S2,S2,S2},
						{S2,S2,S2,S2,M0,M0,S2,S2},
						{S2,S1,S1,S2,S2,S2,S2,S2},
						{S2,S2,S2,S2,S2,S1,S1,S2}};
uint8_t step_len = 8;


float step2[4][8] = {	{M0,S2,S1,S3,S1,S2,M0,S1},
						{M0,M0,M0,M0,M0,M0,M0,M0},
						{M0,M0,M0,M0,M0,M0,M0,M0},
						{M0,M0,M0,M0,M0,M0,M0,M0}};
uint8_t step2_len = 8;


float motion_plan[4][8];
uint8_t current_motion = 0;
uint8_t motion_stage = 0;
uint8_t total_motion_stages = 0;
uint8_t motion_checkpoint_flags = 0;



/* USER ACCESSIBLE FUNCTIONS */

uint8_t motion_in_progress(void)
{
	if(current_motion > 0)
		return 1;
		
	return 0;
}

void begin_motion(uint8_t motion_description)
{	
	// load the initial set of checkpoints into the goal array
	// use the parameter for this to know which array to load
	// the number of steps should also be loaded too (remember to clear this once the goals have been met)
	// from here on, continue motion will be called once the checkpoints have been cleared
	// note that holding a pose also counts as a motion goal (TODO)

	current_motion = motion_description;

	float *motion_directions;

	if(motion_description == MOTION_STAND_UP)
	{
		total_motion_stages = stand_len;
		motion_directions = &stand;
	}
	
	if(motion_description == MOTION_STEP)
	{
		total_motion_stages = step_len;
		motion_directions = &step;	
	}
	
	if(motion_description == MOTION_STEP2)
	{
		total_motion_stages = step2_len;
		motion_directions = &step2;	
	}

	// load the motion program into the array:
	for(uint8_t j = 0; j < 4; j++)
	{
		for(uint8_t i = 0; i < total_motion_stages; i++)
		{
			// pointer arithmetic:
			motion_plan[j][i] = *(motion_directions + (8*j) + i);
			if(MOTION_CONTROL_DEBUG >= 3)
				printf("%.1f ", motion_plan[j][i]);	// DEBUG
		}
		if(MOTION_CONTROL_DEBUG >= 3)
			{printf("\r\n");	_delay_ms(20);}	// DEBUG
	}

	motion_stage = 0;	// start with 0

	if(MOTION_CONTROL_DEBUG >= 1)
		printf("begin motion\r\n");
	
	motion_checkpoint_flags = 0;

	begin_submotion(motion_stage);

}



void clear_actuator_motion_checkpoint(uint8_t act_num)
{
	uint8_t act_ind = act_num - 1;
	motion_checkpoint_flags &=~ (1 << act_ind);

	if(MOTION_CONTROL_DEBUG >= 2)
		printf(" x%u ",act_num);
}

uint8_t motion_subgoal_reached(void)
{	
	if(motion_checkpoint_flags > 0)
		return 0;
	
	return 1;
}

/* INTERNAL FUNCTIONS */

void motion_handler(void)
{
	if(motion_in_progress())
	{
		//printf("<%u", current_motion);
		//printf("-%u>", motion_stage);
		
		// if checkpoints reached, load the next sub-motion
		if(motion_subgoal_reached())
		{
			// increment the grasp stage
			motion_stage++;
				
			if(motion_stage < total_motion_stages)
				begin_submotion(motion_stage);
			else // all the sub-motions are complete, log the motion as complete
			{
				// reset variables for next motion
				motion_stage = 0;
				total_motion_stages = 0;
				current_motion = NO_MOTION;
				if(MOTION_CONTROL_DEBUG >= 1)
					printf("DONE\r\n");
			}
		}
		else
			if(MOTION_CONTROL_DEBUG >= 2)
				printf("*");	// motion subgoal in progress
	}	
}

void begin_submotion(uint8_t motion_stage)
{
	// proceed with this once all the motion checkpoint flags have been cleared
	
	// apply the next set of goals and set the flags, variable:	motion_stage
	uint8_t act_num;


	if(MOTION_CONTROL_DEBUG >= 1)
		printf("CM: %u\r\n", motion_stage);	
	
	if(motion_checkpoint_flags != 0)
		printf("submotion error\r\n");
	
	for(uint8_t i = 0; i < 4; i++)
	{
		// check if there is a different pressure for this appendage for the next stage
		//if(motion_plan[i][motion_stage] != motion_plan[i][motion_stage-1])
		if((motion_stage == 0)||(motion_plan[i][motion_stage] != motion_plan[i][motion_stage-1]))
		{
			if(MOTION_CONTROL_DEBUG >= 2)
				printf("-D%u-",i);
			
			// convert appendage number to actuator number
			if(i == 0)	act_num = WALK_ACT_1;
			if(i == 1)	act_num = WALK_ACT_2;
			if(i == 2)	act_num = WALK_ACT_3;
			if(i == 3)	act_num = WALK_ACT_4;
			
			// if the actuator is connected, set the next pressure goal
			if(is_ADC_pressure_calibrated(act_num))
			{
				motion_checkpoint_flags = make_flag(motion_checkpoint_flags, act_num);
				actuator_set_and_forget(act_num, (float)motion_plan[i][motion_stage]);
			}
		}
	}
}


/* INITIALIZATION */
