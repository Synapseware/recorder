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
#include <LUFA/Drivers/Peripheral/ADC.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/Board/Board.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Misc/RingBuffer.h>
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
#define ADC_port		(PORTB)
#define ADC_pin_reg		(PINB)
#define ADC_clk			(1<<PB6)
#define ADC_data		(1<<PB5)
#define ADC_ss			(1<<PB4)
#define ADC_ddr			(DDRB)


//-------------------------------------------------
// DAC methods
#define DAC_OutStd(v)	((256 * v) / 2.048)
#define DAC_OutHigh(v)	((256 * v) / 4.096)
#define DAC_CLK_DIV		64
void DACOutputHighGain(uint8_t voltage);
void DACOutputStdGain(uint8_t voltage);


/** Maximum audio sample value for the microphone input. */
#define SAMPLE_MAX_RANGE          0xFFFF

/** Maximum ADC range for the microphone input. */
#define ADC_MAX_RANGE             0x03FF


void SetupHardware(void);
static int16_t ADC_ReadSample(void);
static int16_t ADC_GetSample(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

bool CALLBACK_Audio_Device_GetSetEndpointProperty(
	USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
	const uint8_t EndpointProperty,
	const uint8_t EndpointAddress,
	const uint8_t EndpointControl,
	uint16_t* const DataLength,
	uint8_t* Data) ATTR_NON_NULL_PTR_ARG(1);

bool CALLBACK_Audio_Device_GetSetInterfaceProperty(
	USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
	const uint8_t Property,
	const uint8_t EntityAddress,
	const uint16_t Parameter,
	uint16_t* const DataLength,
	uint8_t* Data);

#endif
