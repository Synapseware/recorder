#include "recorder.h"


volatile bool 		_secondsTick 		= false;
volatile bool		_hostConnected		= false;


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;

static void PrintString_P(const char* message)
{
	if (!_hostConnected || NULL == message)
		return;

	fputs_P(message, &USBSerialStream);
}
static void PrintString(const char* message)
{
	if (!_hostConnected || NULL == message)
		return;

	fputs(message, &USBSerialStream);
}
static void PrintChar(char data)
{
	if (!_hostConnected)
		return;

	fputc(data, &USBSerialStream);
}
static void PrintFlush(void)
{
	if (!_hostConnected)
		return;
	CDC_Device_Flush(&VirtualSerial_CDC_Interface);
}


/**  */
void SetupSpi(void)
{
	power_spi_enable();

	SPI_Init(
		SPI_MODE_MASTER |
		SPI_ORDER_MSB_FIRST |
		SPI_SCK_LEAD_RISING |
		SPI_SAMPLE_LEADING |
		SPI_SPEED_FCPU_DIV_16
	);

	// set /SS pin as output
	SPI_ddr |= (SPI_ss);
	SPI_port |= (SPI_ss);
}

/** Configures the I/O pins for the external ADC */
void SetupExternalAdc(void)
{
	ADC_ddr |= (ADC_sck | ADC_ss);
	ADC_ddr &= ~(ADC_data);

	ADC_port |= (ADC_sck | ADC_ss);
}

/**  */
void SetupTimers(void)
{
	// setup timer0 for 1ms interrupts
	{
		// Fcpu / 64 / 250 = 1ms
		// 16MHz / 64 / 250 = 1ms
		// Timer/Counter Control Register A
		TCCR0A  =   (0<<COM0A1)  |
					(0<<COM0A0)	|
					(0<<COM0B1)	|
					(0<<COM0B0)	|
					(1<<WGM01)	|	// CTC
					(0<<WGM00);		// CTC

		// Timer/Counter Control Register B
		TCCR0B  =   (0<<FOC0A)	|
					(0<<FOC0B)	|
					(0<<WGM02)	|	// CTC
					(0<<CS02)	|	// Fcpu/64
					(1<<CS01)	|	// Fcpu/64
					(1<<CS00);		// Fcpu/64

		// Output Compare Register A
		OCR0A   =   250-1;

		// Output Compare Register B
		OCR0B   =   0;

		// Timer/Counter Interrupt Mask Register
		TIMSK0  =   (0<<OCIE0B)	|
					(1<<OCIE0A)	|
					(0<<TOIE0);
	}

	// setup timer1 for whatever ADC conversion rate we want
	{

	}
}

/**  */
void SetupHardware(void)
{
	_secondsTick = false;

	power_all_enable();
	SetupSpi();
	SetupTimers();
	SetupExternalAdc();

	USB_Init();

	// setup debug LED
	DEBUG_LED_ddr |= (DEBUG_LED_msk);

	USB_LED_ddr |= (USB_LED_msk);
	USB_LED_port &= ~(USB_LED_msk);
}

/**  */
int main(void)
{
	SetupHardware();

	/* Create a regular blocking character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateBlockingStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	GlobalInterruptEnable();

	DACOutputStdGain(DAC_OutStd(0.5));

	PrintString_P(PSTR("Audio Recorder and Sampling Prototype\r\n"));
	PrintFlush();

	while(1)
	{
		if (_secondsTick)
		{
			_secondsTick = false;
			PrintChar('.');
			PrintFlush();
		}


		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Writes an output voltage value to the DAC */
void DACOutputHighGain(uint8_t voltage)
{
	uint16_t dacBits = voltage << 4;
	dacBits |=	(0<<13) |	// set high gain
				(1<<12);	// disable shutdown

	SPI_port &= ~(SPI_ss);

	SPI_SendByte(dacBits >> 8);
	SPI_SendByte(dacBits & 0xFF);

	SPI_port |= (SPI_ss);
}

/** Writes an output voltage value to the DAC */
void DACOutputStdGain(uint8_t voltage)
{
	uint16_t dacBits = voltage << 4;
	dacBits |=	(1<<13) |	// clear high gain
				(1<<12);	// disable shutdown

	SPI_port &= ~(SPI_ss);

	SPI_SendByte(dacBits >> 8);
	SPI_SendByte(dacBits & 0xFF);

	SPI_port |= (SPI_ss);
}

/** Reads a 12-bit sample from the ADC */
uint16_t ADC_ReadSample(void)
{
	uint8_t bits = 12;
	uint16_t sample = 0;

	uint8_t sreg = SREG;
	cli();


	// toggle ADC /SS
	ADC_port &= ~(ADC_ss);

	while(bits--)
	{
		// toggle ADC SCK low
		ADC_port &= ~(ADC_sck);

		// shift sample
		sample <<= 1;
		
		// toggle ADC SCK high
		ADC_port |= (ADC_sck);

		// add data
		sample |= ((ADC_port & ADC_data) != 0)
			? 1
			: 0;
	}

	// release ADC /SS
	ADC_port |= (ADC_ss);

	// return the interrupt mask
	SREG = sreg;

	sample &= 0x0FFF;

	return sample;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	// 
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	// 
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	// mark host as connected
	_hostConnected = true;
	USB_LED_port |= (USB_LED_msk);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	// mark host as not connected
	_hostConnected = false;
	USB_LED_port &= ~(USB_LED_msk);
}

/** Millisecond heartbeats */
ISR(TIMER0_COMPA_vect)
{
	static int ticks = 0;

	if (0 == ticks)
	{
		// toggle debug LED
		DEBUG_LED_port |= (DEBUG_LED_msk);
	}
	else
	{
		if (_hostConnected && 15 == ticks)
		{
			DEBUG_LED_port &= ~(DEBUG_LED_msk);
		}
		else if (!_hostConnected && 500 == ticks)
		{
			DEBUG_LED_port &= ~(DEBUG_LED_msk);
		}
	}

	if (ticks++ > 999)
	{
		ticks = 0;
		_secondsTick = true;
	}
}
