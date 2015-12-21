#include "recorder.h"


volatile bool 		_secondsTick 		= false;
volatile bool		_hostConnected		= false;
volatile bool		_greetingSent		= false;
RingBuffer_t		Buffer;
uint8_t				BufferData[1024];


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint 		=
					{
						.Address		= CDC_TX_EPADDR,
						.Size			= CDC_TXRX_EPSIZE,
						.Banks			= 1,
					},
				.DataOUTEndpoint =
					{
						.Address		= CDC_RX_EPADDR,
						.Size			= CDC_TXRX_EPSIZE,
						.Banks			= 1,
					},
				.NotificationEndpoint =
					{
						.Address		= CDC_NOTIFICATION_EPADDR,
						.Size			= CDC_NOTIFICATION_EPSIZE,
						.Banks			= 1,
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
	ADC_ddr |= (ADC_clk | ADC_ss);
	ADC_ddr &= ~(ADC_data);

	ADC_port |= (ADC_clk | ADC_ss);
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

		// Timer/Counter Interrupt Mask Register
		TIMSK0  =   (0<<OCIE0B)	|
					(1<<OCIE0A)	|
					(0<<TOIE0);
	}

	// setup timer1 for whatever ADC conversion rate we want
	{
		// Timer/Counter1 Control Register A
		TCCR1A  =   (0<<COM1A1) |
					(0<<COM1A0) |
					(0<<COM1B1) |
					(0<<COM1B0) |
					(0<<COM1C1) |
					(0<<COM1C0) |
					(0<<WGM11)  |	// ctc
					(0<<WGM10);		// ctc

		// Timer/Counter1 Control Register B
		TCCR1B  =   (0<<ICNC1)  |
					(0<<ICES1)  |
					(0<<WGM13)  |	// ctc
					(1<<WGM12)  |	// ctc
					(0<<CS12)   |	// clk/8
					(1<<CS11)   |	// clk/8
					(0<<CS10);		// clk/8


		// Output Compare Register 1 A
		OCR1A	=	F_CPU / DAC_CLK_DIV / 8000;

		// Timer/Counter1 Interrupt Mask Register
		TIMSK1  =   (0<<ICIE1)  |
					(0<<OCIE1C) |
					(0<<OCIE1B) |
					(1<<OCIE1A) |	// COMPA
					(0<<TOIE1);
	}
}

/**  */
void SetupHardware(void)
{
	_secondsTick = false;
	_hostConnected = false;
	_greetingSent = false;

	RingBuffer_InitBuffer(&Buffer, BufferData, sizeof(BufferData));

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
static void InitialGreeting(void)
{
	PrintString_P(PSTR("Audio Recorder and Sampling Prototype\r\n"));
	PrintString_P(PSTR("November, 2015\r\n"));
	PrintFlush();
}

/**  */
static void PushLatestSampleData(void)
{
	static uint8_t col = 0;

	uint16_t count = RingBuffer_GetCount(&Buffer);
	if (count < 2)
	{
		return;
	}

	char msg[8];
	while (count--)
	{
		uint8_t high = RingBuffer_Remove(&Buffer);
		uint8_t low = RingBuffer_Remove(&Buffer);
		uint16_t sample = high << 8 | low;

		sprintf_P(msg, PSTR("%04x "), sample);
		PrintString(msg);

		col++;
		if (col > 15)
		{
			col = 0;
			PrintString_P(PSTR("\r\n"));
		}
	}

	PrintFlush();
}

/**  */
int main(void)
{
	SetupHardware();
	uint8_t delay = 1;
	uint8_t recordFor = 0;

	/* Create a regular blocking character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateBlockingStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	GlobalInterruptEnable();

	DACOutputStdGain(DAC_OutStd(0.5));

	while(1)
	{
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();

		if (recordFor)
		{
			PushLatestSampleData();
		}

		if (!_secondsTick)
		{
			continue;
		}
		_secondsTick = false;

		if (!_hostConnected)
		{
			// reset the host-connected delay
			delay = 1;
		}
		else
		{
			if (recordFor)
			{
				recordFor--;
			}

			// send greeting if we've hit our host-timeout and the host is connected
			if (delay)
			{
				delay--;
				if (!delay)
				{
					InitialGreeting();
					recordFor = 5;

				}
			}
		}
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

	// toggle ADC /SS
	ADC_port &= ~(ADC_ss);

	while(bits--)
	{
		// toggle ADC SCK low
		ADC_port &= ~(ADC_clk);

		// shift sample
		sample <<= 1;

		// toggle ADC SCK high
		ADC_port |= (ADC_clk);

		// add data
		sample |= ((ADC_pin_reg & ADC_data) > 0)
			? 1
			: 0;
	}

	// release ADC /SS
	ADC_port |= (ADC_ss);

	sample &= 0x0FFF;

	return sample;
}

/** Writes the block of data to the SRAM device */
void SaveSamples(uint8_t * data, int length)
{
	// TBD
	// 
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

/**  */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	/* Check if the DTR line has been asserted - if so, start the target AVR's reset pulse */
	if (CurrentDTRState)
	{
		_hostConnected = true;
		
		if (!_greetingSent)
		{
			_greetingSent = true;
		}

		USB_LED_port |= (USB_LED_msk);
	}
	else
	{
		_hostConnected = false;
		_greetingSent = false;
		USB_LED_port &= ~(USB_LED_msk);
	}
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	// mark host as connected
	//_hostConnected = true;
	//USB_LED_port |= (USB_LED_msk);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	// mark host as not connected
	_hostConnected = false;
	_greetingSent = false;
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

/** ADC sample & save interrupt handler */
ISR(TIMER1_COMPA_vect)
{
	uint16_t sample = ADC_ReadSample();

	RingBuffer_Insert(&Buffer, sample >> 8);
	RingBuffer_Insert(&Buffer, sample & 0xFF);
}
