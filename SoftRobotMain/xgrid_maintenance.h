
#include <avr/io.h>			// for uint8_t
#include <avr/pgmspace.h>	// for pgm_read_byte

#include <util/crc16.h>		// for crc_update

uint16_t get_swarm_id(void);

void calculate_swarm_id(void);

// see comment in implementation file
//uint8_t SP_ReadCalibrationByte( uint8_t index );

uint8_t SP_ReadUserSigRow( uint8_t index );

