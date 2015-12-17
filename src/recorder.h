#ifndef __RECORDER_H__
#define __RECORDER_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "Descriptors.h"
#include "Config/AppConfig.h"

#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/Peripheral/SPI.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Board/Board.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>


//-------------------------------------------------
// Debug LED
#define DEBUG_LED_msk	(1<<PE6)
#define DEBUG_LED_port	(PORTE)
#define DEBUG_LED_ddr	(DDRE)
#define DEBUG_LED_pin	(PINE)


//-------------------------------------------------
// USB status LED
#define USB_LED_msk		(1<<PD7)
#define USB_LED_port	(PORTD)
#define USB_LED_ddr		(DDRD)
#define USB_LED_pin		(PIND)


//-------------------------------------------------
// SPI pins - standard
#define SPI_port		(PORTB)
#define SPI_sck			(1<<PB1)
#define SPI_mosi		(1<<PB2)
#define SPI_miso		(1<<PB3)
#define SPI_ss			(1<<PB0)
#define SPI_ddr			(DDRB)


//-------------------------------------------------
// External SPI ADC
#define ADC_port		(PORTD)
#define ADC_sck			(1<<PD0)
#define ADC_data		(1<<PD1)
#define ADC_ss			(1<<PD2)
#define ADC_ddr			(DDRD)


//-------------------------------------------------
// DAC methods
#define DAC_OutStd(v)	((256 * v) / 2.048)
#define DAC_OutHigh(v)	((256 * v) / 4.096)
void DACOutputHighGain(uint8_t voltage);
void DACOutputStdGain(uint8_t voltage);



#endif
