
#include "xgrid_maintenance.h"


// Build information
extern char   __BUILD_DATE;
extern char   __BUILD_NUMBER;

uint16_t swarm_id;	// facilitated by main.cpp

uint16_t get_swarm_id(void)
{
	return swarm_id;
}


void calculate_swarm_id()
{
	// note: swarm_id is uint16_t and is global
	uint32_t b = 0;
	uint32_t crc = 0;
	//uint8_t t;
	
	// calculate local id
	// simply crc of user sig row
	// likely to be unique and constant for each chip
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (uint32_t i = 0x08; i <= 0x15; i++)
	{
		b = pgm_read_byte_far(i);
		//b = PGM_READ_BYTE(i);
		crc = _crc16_update(crc, b);
	}
	
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	
	swarm_id = crc;
	
	if(swarm_id < 0)
	swarm_id*=-1;
}

/*
for some reason, this implementation is unrecognized, ATMEL has its own implementation
in sp_driver.h/.S which has also been included in the xboot folder with the xgrid files

// Production signature row access
uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return result;
}*/

// User signature row access
uint8_t SP_ReadUserSigRow( uint8_t index )
{
	uint8_t result;
	NVM_CMD = NVM_CMD_READ_USER_SIG_ROW_gc;
	result = pgm_read_byte(index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return result;
}
