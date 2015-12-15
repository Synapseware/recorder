#ifndef __RECORDER_H__
#define __RECORDER_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <math.h>



//-------------------------------------------------
// Debug LED
#define DEBUG_LED_msk	(1<<PE6)
#define DEBUG_LED_port	(PORTE)
#define DEBUG_LED_ddr	(DDRE)


//-------------------------------------------------
// SPI pins - standard
#define SPI_port		(PORTB)
#define SPI_sck			(1<<PB1)
#define SPI_mosi		(1<<PB2)
#define SPI_miso		(1<<PB3)
#define SPI_ss			(1<<PB0)
#define SPI_ddr			(DDRB)




void DACOutputHighGain(uint8_t voltage);



#endif
