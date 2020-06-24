/******************************************************************************
***************************Intermediate driver layer***************************
* | file      	:	OLED_Driver.c
* |	version		:	V1.0
* | date		:	2017-11-09
* | function	:	SSD1327 Drive function
	
note:
Image scanning:
Please use progressive scanning to generate images or fonts			
******************************************************************************/
#include "OLED/OLED_Driver.h"
#include "main.h"
#include <stdio.h>

COLOR Buffer[OLED_WIDTH / 2 * OLED_HEIGHT];
OLED_DIS sOLED_DIS;
/*******************************************************************************
function:
			Hardware reset
*******************************************************************************/
static void OLED_Reset(void)
{
    OLED_RST_1;
    Driver_Delay_ms(100);
    OLED_RST_0;
    Driver_Delay_ms(100);
    OLED_RST_1;
    Driver_Delay_ms(100);
}

/*******************************************************************************
function:
		Write register address and data
*******************************************************************************/
void OLED_WriteReg(uint8_t Reg)
{
    OLED_DC_0;
    OLED_CS_0;
    SPI4W_Write_Byte(Reg);
    OLED_CS_1;
}

void OLED_WriteData(uint8_t Data)
{
    OLED_DC_1;
    OLED_CS_0;
    SPI4W_Write_Byte(Data);
    OLED_CS_1;
}

/*******************************************************************************
function:
		Common register initialization
*******************************************************************************/
static void OLED_InitReg(void)
{
    OLED_WriteReg(0xae);//--turn off oled panel

    OLED_WriteReg(0x15);    //   set column address
    OLED_WriteReg(0x00);    //  start column   0
    OLED_WriteReg(0x7f);    //  end column   127

    OLED_WriteReg(0x75);    //   set row address
    OLED_WriteReg(0x00);    //  start row   0
    OLED_WriteReg(0x7f);    //  end row   127

    OLED_WriteReg(0x81);  // set contrast control
    OLED_WriteReg(0x80);

    OLED_WriteReg(0xa0);    // gment remap
    OLED_WriteReg(0x51);   //51

    OLED_WriteReg(0xa1);  // start line
    OLED_WriteReg(0x00);

    OLED_WriteReg(0xa2);  // display offset
    OLED_WriteReg(0x00);

    OLED_WriteReg(0xa4);    // rmal display
    OLED_WriteReg(0xa8);    // set multiplex ratio
    OLED_WriteReg(0x7f);

    OLED_WriteReg(0xb1);  // set phase leghth
    OLED_WriteReg(0xf1);

    OLED_WriteReg(0xb3);  // set dclk
    OLED_WriteReg(0x00);  //80Hz:0xc1 90Hz:0xe1   100Hz:0x00   110Hz:0x30 120Hz:0x50   130Hz:0x70     01

    OLED_WriteReg(0xab);  //
    OLED_WriteReg(0x01);  //

    OLED_WriteReg(0xb6);  // set phase leghth
    OLED_WriteReg(0x0f);

    OLED_WriteReg(0xbe);
    OLED_WriteReg(0x0f);

    OLED_WriteReg(0xbc);
    OLED_WriteReg(0x08);

    OLED_WriteReg(0xd5);
    OLED_WriteReg(0x62);

    OLED_WriteReg(0xfd);
    OLED_WriteReg(0x12);

}

/********************************************************************************
function:	Set the display scan and color transfer modes
parameter:
		Scan_dir   :   Scan direction
		Colorchose :   RGB or GBR color format
********************************************************************************/
void OLED_SetGramScanWay(OLED_SCAN_DIR Scan_dir)
{
    //Get the screen scan direction
    sOLED_DIS.OLED_Scan_Dir = Scan_dir;

    //Get GRAM and OLED width and height
    if(Scan_dir == L2R_U2D || Scan_dir == L2R_D2U || Scan_dir == R2L_U2D || Scan_dir == R2L_D2U) {
        sOLED_DIS.OLED_Dis_Column	= OLED_WIDTH;
        sOLED_DIS.OLED_Dis_Page = OLED_HEIGHT;
        sOLED_DIS.OLED_X_Adjust = OLED_X;
        sOLED_DIS.OLED_Y_Adjust = OLED_Y;
    } else {
        sOLED_DIS.OLED_Dis_Column	= OLED_HEIGHT;
        sOLED_DIS.OLED_Dis_Page = OLED_WIDTH;
        sOLED_DIS.OLED_X_Adjust = OLED_Y;
        sOLED_DIS.OLED_Y_Adjust = OLED_X;
    }
}

/********************************************************************************
function:
			initialization
********************************************************************************/
void OLED_Init(OLED_SCAN_DIR OLED_ScanDir)
{
    //Hardware reset
    OLED_Reset();

    //Set the initialization register
    OLED_InitReg();

    //Set the display scan and color transfer modes
    OLED_SetGramScanWay(OLED_ScanDir );
    Driver_Delay_ms(200);

    //Turn on the OLED display
    OLED_WriteReg(0xAF);
}

/********************************************************************************
function:	Set the display point(Xpoint, Ypoint)
parameter:
		xStart :   X direction Start coordinates
		xEnd   :   X direction end coordinates
********************************************************************************/
void OLED_SetCursor(POINT Xpoint, POINT Ypoint)
{
    if((Xpoint > sOLED_DIS.OLED_Dis_Column) || (Ypoint > sOLED_DIS.OLED_Dis_Page))
        return;

    OLED_WriteReg(0x15);
    OLED_WriteReg(Xpoint);
    OLED_WriteReg(Xpoint);

    OLED_WriteReg(0x75);
    OLED_WriteReg(Ypoint);
    OLED_WriteReg(Ypoint);
}

/********************************************************************************
function:	Set the display Window(Xstart, Ystart, Xend, Yend)
parameter:
		xStart :   X direction Start coordinates
		Ystart :   Y direction Start coordinates
		Xend   :   X direction end coordinates
		Yend   :   Y direction end coordinates
********************************************************************************/
void OLED_SetWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend)
{
    if((Xstart > sOLED_DIS.OLED_Dis_Column) || (Ystart > sOLED_DIS.OLED_Dis_Page) ||
       (Xend > sOLED_DIS.OLED_Dis_Column) || (Yend > sOLED_DIS.OLED_Dis_Page))
        return;

    OLED_WriteReg(0x15);
    OLED_WriteReg(Xstart);
    OLED_WriteReg(Xend - 1);

    OLED_WriteReg(0x75);
    OLED_WriteReg(Ystart);
    OLED_WriteReg(Yend - 1);
}

/********************************************************************************
function:	Set show color
parameter:
		Color  :   Set show color,16-bit depth
********************************************************************************/
//static void OLED_SetColor(LENGTH Dis_Width, LENGTH Dis_Height, COLOR Color ){
void OLED_SetColor(POINT Xpoint, POINT Ypoint, COLOR Color)
{
    if(Xpoint > sOLED_DIS.OLED_Dis_Column || Ypoint > sOLED_DIS.OLED_Dis_Page) {
        return;
    }
    //1 byte control two points
    if(Xpoint % 2 == 0) {
        Buffer[Xpoint / 2 + Ypoint * 64] = (Color << 4) | Buffer[Xpoint / 2 + Ypoint * 64];
    } else {
        Buffer[Xpoint / 2 + Ypoint * 64] = (Color & 0x0f) | Buffer[Xpoint / 2 + Ypoint * 64];
    }
}

/********************************************************************************
function:
			Clear screen
********************************************************************************/
void OLED_Clear(COLOR Color)
{
    unsigned int i,m;
    //OLED_SetWindow(0, 0, sOLED_DIS.OLED_Dis_Column, sOLED_DIS.OLED_Dis_Page);
    for(i = 0; i < sOLED_DIS.OLED_Dis_Page; i++) {
        for(m = 0; m < (sOLED_DIS.OLED_Dis_Column / 2); m++) {
            Buffer[i * (sOLED_DIS.OLED_Dis_Column / 2) + m] = Color | (Color << 4);
        }
    }
}

/********************************************************************************
function:	Update all memory to LCD
********************************************************************************/
void OLED_Display(void)
{
    uint16_t page, Column;
    COLOR *pBuf = (COLOR *)Buffer;

    OLED_SetWindow(0, 0, sOLED_DIS.OLED_Dis_Column, sOLED_DIS.OLED_Dis_Page);
    //write data
    for (page = 0; page < sOLED_DIS.OLED_Dis_Page; page++) {
        for(Column = 0; Column < sOLED_DIS.OLED_Dis_Column / 2; Column++ ) {
            OLED_WriteData(*pBuf);
            pBuf++;
        }
    }
}

/********************************************************************************
function:
			Clear Window
********************************************************************************/
void OLED_ClearWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color)
{
    uint16_t i,m, Xpoint, Ypoint;
	Xpoint = (Xend - Xstart) / 2;
	Ypoint = Yend - Ystart;
	
    uint16_t Num = Xstart + Ystart * (sOLED_DIS.OLED_Dis_Column / 2);
    for(i = 0; i < Ypoint; i++) {
        for(m = 0; m < Xpoint; m++) {
            Buffer[Num + m] = 0x00;
        }
		Num = Xstart + (Ystart + i + 1) * (sOLED_DIS.OLED_Dis_Column / 2);
    }
}

/********************************************************************************
function:	Update Window memory to LCD
********************************************************************************/
void OLED_DisWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend)
{
	uint16_t page, Column, Xpoint, Ypoint;
	Xpoint = (Xend - Xstart) / 2;
	Ypoint = Yend - Ystart;
    OLED_SetWindow(Xstart, Ystart, Xend, Yend);
	
	//write data
    COLOR *pBuf = (COLOR *)Buffer + Xstart + Ystart * (sOLED_DIS.OLED_Dis_Column / 2);
    for (page = 0; page < Ypoint; page++) {
        for(Column = 0; Column < Xpoint; Column++ ) {
            OLED_WriteData(*pBuf);
            pBuf++;
        }
		pBuf = (COLOR *)Buffer + Xstart + (Ystart + page + 1) * (sOLED_DIS.OLED_Dis_Column / 2);
	}
}

void OLED_DisplayOFF(void)
{
	/* Panel OFF */
	OLED_WriteReg(0xAE);

	/* Internal regulator OFF */
	OLED_WriteReg(0xAB);
	OLED_WriteReg(0x00);
}

void OLED_DisplayON(void)
{
	/* Internal regulator ON */
	OLED_WriteReg(0xAB);
	OLED_WriteReg(0x01);

	/* Panel ON */
	OLED_WriteReg(0xAF);
}
