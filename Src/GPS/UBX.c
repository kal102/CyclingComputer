#include "GPS/UART.h"

/* Commands */
#define RXM_PMREQ 0x0241

void UBX_formatFrame(uint8_t *UBXframe, uint16_t UBXcommand, uint8_t UBXdataLength, uint8_t *UBXdata)
{
	uint16_t index;
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	
	// Input constant header and variable parameters.
	UBXframe[0] = 0xB5;
	UBXframe[1] = 0x62;
	UBXframe[2] = (UBXcommand & 0xFF00) >> 8;
	UBXframe[3] = (UBXcommand & 0x00FF);
	UBXframe[4] = UBXdataLength;
	UBXframe[5] = UBXdataLength >> 8;
	
	// Input UBXdata into UBXframe.
	for(index = 0; index < UBXdataLength; index++)
	{
		UBXframe[6 + index] = UBXdata[index];
	}
	
	// Calculate checksum and input it into UBXframe.
	for(index = 2; index < (6 + UBXdataLength); index++)
	{
		CK_A = CK_A + UBXframe[index];
		CK_B = CK_B + CK_A;
	}

	UBXframe[6 + UBXdataLength]	= CK_A;
	UBXframe[6 + UBXdataLength + 1] = CK_B;
}

uint8_t UBX_scanFrame(uint8_t *UBXframe, uint16_t *UBXcommand, uint8_t *UBXdataLength, uint8_t *UBXdata)
{
	uint16_t index, dataLength;
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	
	// Firstly, check tha header is correct.
	if((UBXframe[0] != 0xB5) || (UBXframe[1] != 0x62))
	{
		return 1; // Error 1, incorrect header.
	}
	
	// If header is ok, check that checksum is correct.
	dataLength = (UBXframe[5] << 8) + UBXframe[4];
	for(index = 2; index < (6 + dataLength); index++)
	{
		CK_A = CK_A + UBXframe[index];
		CK_B = CK_B + CK_A;
	}
	if((CK_A != (uint16_t)UBXframe[6 + dataLength]) || (CK_B != (uint16_t)UBXframe[7 + dataLength]))
	{
		return 2; // Error 2, incorrect checksum.
	}
	
	// If checksum is ok, copy data and parameters from UBXstring.
	*UBXcommand = (UBXframe[2] << 8);
	*UBXcommand += UBXframe[3];
	*UBXdataLength = dataLength;
	for(index = 0; index < dataLength; index++)
	{
		UBXdata[index] = UBXframe[6 + index];
	}
	return 0;
}

static inline uint8_t UBX_calculateFrameSize(uint8_t UBXdataLength)
{
	return (6 + UBXdataLength + 2);
}

void UBX_powerSavingON(void)
{
	uint8_t UBXframe[16];
	uint8_t UBXdataLength = 8;
	uint8_t UBXdata[8] = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
	uint8_t UBXframeSize = UBX_calculateFrameSize(UBXdataLength);

	UBX_formatFrame(UBXframe, RXM_PMREQ, UBXdataLength, UBXdata);

	UART_sendArray(UBXframe, UBXframeSize);
	while(UART_getTxFlag());
}

void UBX_powerSavingOFF(void)
{
	uint8_t byte = 0xFF;

	UART_sendByte(byte);
}
