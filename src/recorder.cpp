#include "recorder.h"




/**  */
void SetupSpi(void)
{
	// SPI Control Register
	SPCR	=	(0<<SPIE)	|   // Bit 7 – SPIE: SPI Interrupt Enable
				(1<<SPE)	|   // Bit 6 – SPE: SPI Enable
				(0<<DORD)	|   // Bit 5 – DORD: Data Order
				(1<<MSTR)	|   // Bit 4 – MSTR: Master/Slave Select
				(0<<CPOL)	|   // Bit 3 – CPOL: Clock Polarity
				(0<<CPHA)	|   // Bit 2 – CPHA: Clock Phase
				(0<<SPR1)	|	// Bit 1 - SPR1: SPI Clock Rate Select
				(1<<SPR0);	  	// Bit 0 - SPR0: SPI Clock Rate Select
	
	// SPI Status Register
	SPSR	=	(0<<SPIF)	|   // Bit 7 – SPIF: SPI Interrupt Flag
				(0<<WCOL)	|   // Bit 6 – WCOL: Write COLlision Flag
				(0<<SPI2X);		// Bit 0 – SPI2X: Double SPI Speed Bit

	SPI_ddr |=	(SPI_sck)	|	// SCK
				(SPI_mosi)	|	// MOSI
				(SPI_ss);		// SS

	SPI_ddr &=	~(SPI_miso);	// MISO
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

	while(1)
	{

	}
}


/** Writes an output voltage value to the DAC */
void DACOutputHighGain(uint8_t voltage)
{
	uint16_t dacBits = voltage << 4;
	dacBits |=	(1<<13) |	// set high gain
				(1<<12);	// disable shutdown

	SPI_ddr
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
	}
}