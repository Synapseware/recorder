#include "raw.h"


static int HexCharToInt(int data)
{
	if (data >= '0' && data <= '9')
		return (data - '0');
	else if (data >= 'A' && data <= 'F')
		return (data - 'A');
	else if (data >= 'a' && data <= 'f')
		return (data - 'a');
	else if (data < 16)
		return data;
}

/**  */
static void convertFile(const char* src, const char* dest)
{
	cout << "Converting hex data in " << src << " to output file " << dest << endl;

	FILE * srcFile = fopen(src, "r");
	FILE * dstFile = fopen(dest, "w");


	int converted = 0;
	while (true)
	{
		int high = fgetc(srcFile);
		if (high < 0)
		{
			break;
		}
		int low = fgetc(srcFile);
		if (low < 0)
		{
			break;
		}

		int sample = high << 8 | low;

		fputc(sample / 4, dstFile);

		converted++;
	}
	cout << "Converted " << converted << " bytes." << endl;

	fclose(srcFile);
	fclose(dstFile);

	/*
	for (int i = 0; i < FLASH_TOTAL_SIZE; i++)
	{
		fputc(0xFF, file);
	}
	fclose(file);
	*/
}


/** main */
int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		cout << "Missing parameters.  Aborting." << endl;
		return -1;
	}

	char *srcFile = argv[1];
	char *destFile = argv[2];

	convertFile(srcFile, destFile);

	return 0;
}
