// SRburners.h

#ifndef __SRBURNERS_H__
#define __SRBURNERS_H__

#include <avr/io.h>		// fixes compile error "unknown type name 'uint8_t'"

#ifdef __cplusplus
extern "C"
{
#endif

#define BURNER_PORT 				PORTB
#define BURNER1_PIN_bm				PIN4_bm
#define BURNER2_PIN_bm				PIN5_bm


void burners_init(void);

void burner_on(uint8_t burner_num);

void burner_off(uint8_t burner_num);




#ifdef __cplusplus
}
#endif

#endif