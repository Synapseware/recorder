#include "recorder.h"


volatile bool _secondsTick = false;


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
}

/**  */
void Setup(void)
{
	_secondsTick = false;

	power_all_enable();
	SetupSpi();
	SetupTimers();

	// setup debug LED
	DEBUG_LED_ddr |= (DEBUG_LED_msk);

	sei();
}

/**  */
int main(void)
{
	Setup();

	uint8_t voltage = 0;
	//uint8_t delay = 5;

	while(1)
	{
		if (_secondsTick)
		{
			//delay--;
			//if (!delay)
			{
			//	delay = 5;

				DACOutputHighGain(voltage);
				voltage += 16;
			}

			_secondsTick = false;
		}
	}
}


/** Writes an output voltage value to the DAC */
void DACOutputHighGain(uint8_t voltage)
{
	uint16_t dacBits = voltage << 4;
	dacBits |=	(1<<13) |	// set high gain
				(1<<12);	// disable shutdown

	SPI_port &= ~(SPI_ss);

	SPI_SendByte(dacBits >> 8);
	SPI_SendByte(dacBits & 0xFF);

	SPI_port |= (SPI_ss);
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
	else if (15 == ticks)
	{
		DEBUG_LED_port &= ~(DEBUG_LED_msk);
	}

	if (ticks++ > 999)
	{
		ticks = 0;
		_secondsTick = true;
	}
}