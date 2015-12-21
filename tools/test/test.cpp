#include "test.h"


/**  */
static void testAdcSampling(void)
{
	uint16_t sample = 0;
	uint8_t bits = 12;

	srand(time(NULL));

	while(bits--)
	{
		sample <<= 1;

		sample |= ((rand() % 10) >= 5)
			? 1
			: 0;
	}

	sample &= 0x0FFF;

	char msg[256];
	sprintf(msg, "Sample: 0x%04x", sample);
	cout << msg << endl;
	sprintf(msg, "  high: 0x%02x", sample >> 8);
	cout << msg << endl;
	sprintf(msg, "   low: 0x%02x", sample & 0xFF);
	cout << msg << endl;
}


/** main */
int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		cout << "Missing test name.  Aborting." << endl;
		return -1;
	}

	char *method = argv[1];

	for (int i = 1; i < argc; i++)
	{
		if (0 == strncmp("-m", argv[i], 2))
			method = argv[++i];
	}

	if (0 == strncmp("adc", method, 9))
	{
		testAdcSampling();
		return 0;
	}

	cout << "Unknown method: " << method << endl;
}
