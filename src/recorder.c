#include "recorder.h"


volatile bool 		_secondsTick 		= false;
volatile bool		_hostConnected		= false;
volatile bool		_readSample			= false;
volatile int16_t	_lastSample			= 1023;

/** Current audio sampling frequency of the streaming audio endpoint. */
static uint32_t CurrentAudioSampleFrequency = 8000;


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_Audio_Device_t Microphone_Audio_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_AudioControl,
				.StreamingInterfaceNumber = INTERFACE_ID_AudioStream,
				.DataINEndpoint           =
					{
						.Address          = AUDIO_STREAM_EPADDR,
						.Size             = AUDIO_STREAM_EPSIZE,
						.Banks            = 2,
					},
			},
	};

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

	DACOutputStdGain(DAC_OutStd(0.5));
}

/**  */
void SetupTimers(void)
{
	// setup timer0 for 1ms interrupts
	/*{
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
	}*/

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
					(0<<CS12)   |	// clk/64
					(1<<CS11)   |	// clk/64
					(1<<CS10);		// clk/64


		// Output Compare Register 1 A
		OCR1A	=	F_CPU / DAC_CLK_DIV / 1000;

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

	power_all_enable();
	SetupSpi();
	SetupTimers();
	//SetupExternalAdc();

#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	ADC_Init(ADC_FREE_RUNNING | ADC_PRESCALE_32);
	ADC_SetupChannel(MIC_IN_ADC_CHANNEL);

	USB_Init();

	ADC_StartReading(ADC_REFERENCE_AVCC | ADC_RIGHT_ADJUSTED | ADC_GET_CHANNEL_MASK(MIC_IN_ADC_CHANNEL));

	// setup debug LED
	DEBUG_LED_ddr |= (DEBUG_LED_msk);

	USB_LED_ddr |= (USB_LED_msk);
	USB_LED_port &= ~(USB_LED_msk);
}

/**  */
int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	while(1)
	{
		if (_readSample)
		{
			_lastSample = ADC_ReadSample();
			_readSample = false;
		}

		Audio_Device_USBTask(&Microphone_Audio_Interface);
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
static int16_t ADC_ReadSample(void)
{
	uint8_t bits = 12;
	int16_t sample = 0;

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

	return sample; // - 1023;
}

/**  */
static int16_t ADC_GetResultOld(void)
{
	#ifdef USE_TEST_DATA
		static uint16_t index = 0;

		uint8_t sample = pgm_read_byte(&TEST_SOUND[index++]);
		if (index >= TEST_SOUND_LEN)
			index = 0;

		return sample;
	#else
		return _lastSample;
	#endif
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	// mark host as connected
	_hostConnected = true;
	USB_LED_port |= (USB_LED_msk);

	/* Sample reload timer initialization */
	TIMSK0  = (1 << OCIE0A);
	OCR0A   = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
	TCCR0A  = (1 << WGM01);  // CTC mode
	TCCR0B  = (1 << CS01);   // Fcpu/8 speed
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	// mark host as not connected
	_hostConnected = false;
	USB_LED_port &= ~(USB_LED_msk);

	/* Stop the sample reload timer */
	TCCR0B = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	// 
	Audio_Device_ConfigureEndpoints(&Microphone_Audio_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	// 
	Audio_Device_ProcessControlRequest(&Microphone_Audio_Interface);
}

/** Audio class driver callback for the setting and retrieval of streaming endpoint properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio endpoints.
 *
 *  When the DataLength parameter is NULL, this callback should only indicate whether the specified operation is valid for
 *  the given endpoint index, and should return as fast as possible. When non-NULL, this value may be altered for GET operations
 *  to indicate the size of the retrieved data.
 *
 *  \note The length of the retrieved data stored into the Data buffer on GET operations should not exceed the initial value
 *        of the \c DataLength parameter.
 *
 *  \param[in,out] AudioInterfaceInfo  Pointer to a structure containing an Audio Class configuration and state.
 *  \param[in]     EndpointProperty    Property of the endpoint to get or set, a value from Audio_ClassRequests_t.
 *  \param[in]     EndpointAddress     Address of the streaming endpoint whose property is being referenced.
 *  \param[in]     EndpointControl     Parameter of the endpoint to get or set, a value from Audio_EndpointControls_t.
 *  \param[in,out] DataLength          For SET operations, the length of the parameter data to set. For GET operations, the maximum
 *                                     length of the retrieved data. When NULL, the function should return whether the given property
 *                                     and parameter is valid for the requested endpoint without reading or modifying the Data buffer.
 *  \param[in,out] Data                Pointer to a location where the parameter data is stored for SET operations, or where
 *                                     the retrieved data is to be stored for GET operations.
 *
 *  \return Boolean \c true if the property get/set was successful, \c false otherwise
 */
bool CALLBACK_Audio_Device_GetSetEndpointProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
                                                  const uint8_t EndpointProperty,
                                                  const uint8_t EndpointAddress,
                                                  const uint8_t EndpointControl,
                                                  uint16_t* const DataLength,
                                                  uint8_t* Data)
{
	/* Check the requested endpoint to see if a supported endpoint is being manipulated */
	if (EndpointAddress == Microphone_Audio_Interface.Config.DataINEndpoint.Address)
	{
		/* Check the requested control to see if a supported control is being manipulated */
		if (EndpointControl == AUDIO_EPCONTROL_SamplingFreq)
		{
			switch (EndpointProperty)
			{
				case AUDIO_REQ_SetCurrent:
					/* Check if we are just testing for a valid property, or actually adjusting it */
					if (DataLength != NULL)
					{
						/* Set the new sampling frequency to the value given by the host */
						CurrentAudioSampleFrequency = (((uint32_t)Data[2] << 16) | ((uint32_t)Data[1] << 8) | (uint32_t)Data[0]);

						/* Adjust sample reload timer to the new frequency */
						OCR0A = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
					}

					return true;
				case AUDIO_REQ_GetCurrent:
					/* Check if we are just testing for a valid property, or actually reading it */
					if (DataLength != NULL)
					{
						*DataLength = 3;

						Data[2] = (CurrentAudioSampleFrequency >> 16);
						Data[1] = (CurrentAudioSampleFrequency >> 8);
						Data[0] = (CurrentAudioSampleFrequency &  0xFF);
					}

					return true;
			}
		}
	}

	return false;
}

/** Audio class driver callback for the setting and retrieval of streaming interface properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio interfaces.
 *
 *  When the DataLength parameter is NULL, this callback should only indicate whether the specified operation is valid for
 *  the given entity and should return as fast as possible. When non-NULL, this value may be altered for GET operations
 *  to indicate the size of the retrieved data.
 *
 *  \note The length of the retrieved data stored into the Data buffer on GET operations should not exceed the initial value
 *        of the \c DataLength parameter.
 *
 *  \param[in,out] AudioInterfaceInfo  Pointer to a structure containing an Audio Class configuration and state.
 *  \param[in]     Property            Property of the interface to get or set, a value from Audio_ClassRequests_t.
 *  \param[in]     EntityAddress       Address of the audio entity whose property is being referenced.
 *  \param[in]     Parameter           Parameter of the entity to get or set, specific to each type of entity (see USB Audio specification).
 *  \param[in,out] DataLength          For SET operations, the length of the parameter data to set. For GET operations, the maximum
 *                                     length of the retrieved data. When NULL, the function should return whether the given property
 *                                     and parameter is valid for the requested endpoint without reading or modifying the Data buffer.
 *  \param[in,out] Data                Pointer to a location where the parameter data is stored for SET operations, or where
 *                                     the retrieved data is to be stored for GET operations.
 *
 *  \return Boolean \c true if the property GET/SET was successful, \c false otherwise
 */
bool CALLBACK_Audio_Device_GetSetInterfaceProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
                                                   const uint8_t Property,
                                                   const uint8_t EntityAddress,
                                                   const uint16_t Parameter,
                                                   uint16_t* const DataLength,
                                                   uint8_t* Data)
{
	/* No audio interface entities in the device descriptor, thus no properties to get or set. */
	return false;
}

/** ISR to handle the reloading of the data endpoint with the next sample. */
ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
	uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();

	/* Check that the USB bus is ready for the next sample to write */
	if (Audio_Device_IsReadyForNextSample(&Microphone_Audio_Interface))
	{
		/* Audio sample is ADC value scaled to fit the entire range */
		int16_t AudioSample = ((SAMPLE_MAX_RANGE / ADC_MAX_RANGE) * ADC_GetResult());
		#ifndef USE_TEST_DATA
			//_readSample = true;
		#endif

		#if defined(MICROPHONE_BIASED_TO_HALF_RAIL)
		/* Microphone is biased to half rail voltage, subtract the bias from the sample value */
		AudioSample -= (SAMPLE_MAX_RANGE / 2);
		#endif

		Audio_Device_WriteSample16(&Microphone_Audio_Interface, AudioSample);
	}

	Endpoint_SelectEndpoint(PrevEndpoint);
}

/** Millisecond heartbeats */
ISR(TIMER1_COMPA_vect)
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
