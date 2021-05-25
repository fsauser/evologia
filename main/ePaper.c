#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/uart.h"

#define epaper_c

#include "include.h"

//-----------------------------------------------------------
// Load Image Area process
//-----------------------------------------------------------
void ePaperHostAreaPackedPixelWrite(IT8951LdImgInfo* pstLdImgInfo,IT8951AreaImgInfo* pstAreaImgInfo)
{
	uint32_t j;
	//Source buffer address of Host
	uint16_t* pusFrameBuf = (uint16_t*)pstLdImgInfo->ulStartFBAddr;

	//Set Image buffer(IT8951) Base address
	it8951SetImgBufBaseAddr(pstLdImgInfo->ulImgBufBaseAddr);
	//Send Load Image start Cmd
	it8951LoadImgAreaStart(pstLdImgInfo , pstAreaImgInfo);
	//Host Write Data
	for(j=0;j< pstAreaImgInfo->usHeight;j++)
	{
		/*
		 for(i=0;i< pstAreaImgInfo->usWidth/2;i++)
			{
					//Write a Word(2-Bytes) for each time
					//it8951WriteData(*pusFrameBuf);
					it8951WriteNData((uint8_t*)(pusFrameBuf), 2);
					pusFrameBuf++;
			}
			*/
		it8951WriteNData((uint8_t*)(pusFrameBuf), 2*(pstAreaImgInfo->usWidth/4));
		pusFrameBuf+=(pstAreaImgInfo->usWidth/4);
		it8951WriteNData((uint8_t*)(pusFrameBuf), 2*(pstAreaImgInfo->usWidth/4));
		pusFrameBuf+=(pstAreaImgInfo->usWidth/4);;
	}
	//Send Load Img End Command
	it8951LoadImgEnd();
}
//-----------------------------------------------------------
// Load Image Area process Letter
//-----------------------------------------------------------
void ePaperHostAreaPackedPixelWriteNbr(IT8951LdImgInfo* pstLdImgInfo,IT8951AreaImgInfo* pstAreaImgInfo)
{
	uint32_t j;
	//Source buffer address of Host
	uint16_t* pusFrameBuf = (uint16_t*)pstLdImgInfo->ulStartFBAddr;

	//Set Image buffer(IT8951) Base address
	it8951SetImgBufBaseAddr(pstLdImgInfo->ulImgBufBaseAddr);
	//Send Load Image start Cmd
	it8951LoadImgAreaStart(pstLdImgInfo , pstAreaImgInfo);
	//Host Write Data
	for(j=0;j< pstAreaImgInfo->usHeight;j++)
	{
		it8951WriteNData((uint8_t*)(pusFrameBuf), 2*(pstAreaImgInfo->usWidth/2));
		pusFrameBuf+=(pstAreaImgInfo->usWidth/2);
	}
	//Send Load Img End Command
	it8951LoadImgEnd();
}
//-----------------------------------------------------------
//Display functions 3---Application for Display panel Area
//-----------------------------------------------------------
void it8951DisplayArea(uint16_t usX, uint16_t usY, uint16_t usW, uint16_t usH, uint16_t usDpyMode)
{
	//Send I80 Display Command (User defined command of IT8951)
	it8951WriteCmdCode(USDEF_I80_CMD_DPY_AREA); //0x0034
	//Write arguments
	it8951WriteData(usX);
	it8951WriteData(usY);
	it8951WriteData(usW);
	it8951WriteData(usH);
	it8951WriteData(usDpyMode);
}
//Display Area with bitmap on EPD
//-----------------------------------------------------------
// Display Function 4---for Display Area for 1-bpp mode format
//   the bitmap(1bpp) mode will be enable when Display
//   and restore to Default setting (disable) after displaying finished
//-----------------------------------------------------------
void ePaperDisplayArea1bpp(uint16_t usX, uint16_t usY, uint16_t usW, uint16_t usH, uint16_t usDpyMode, uint8_t ucBGGrayVal, uint8_t ucFGGrayVal)
{
    //Set Display mode to 1 bpp mode - Set 0x18001138 Bit[18](0x1800113A Bit[2])to 1
    it8951WriteReg(UP1SR+2, it8951ReadReg(UP1SR+2) | (1<<2));
    
    //Set BitMap color table 0 and 1 , => Set Register[0x18001250]:
    //Bit[7:0]: ForeGround Color(G0~G15)  for 1
    //Bit[15:8]:Background Color(G0~G15)  for 0
    it8951WriteReg(BGVR, (ucBGGrayVal<<8) | ucFGGrayVal);
    
    //Display
    it8951DisplayArea( usX, usY, usW, usH, usDpyMode);
    it8951WaitForDisplayReady();
    
    //Restore to normal mode
    it8951WriteReg(UP1SR+2, it8951ReadReg(UP1SR+2) & ~(1<<2));
}
//-------------------------------------------------------------------------------------------------------------
// 	Command - 0x0037 for Display Base addr by User 
//  uint32_t ulDpyBufAddr - Host programmer need to indicate the Image buffer address of IT8951
//                                         In current case, there is only one image buffer in IT8951 so far.
//                                         So Please set the Image buffer address you got  in initial stage.
//                                         (gulImgBufAddr by Get device information 0x0302 command)
//
//-------------------------------------------------------------------------------------------------------------
void ePaperDisplayAreaBuf(uint16_t usX, uint16_t usY, uint16_t usW, uint16_t usH, uint16_t usDpyMode, uint32_t ulDpyBufAddr)
{
    //Send I80 Display Command (User defined command of IT8951)
    it8951WriteCmdCode(USDEF_I80_CMD_DPY_BUF_AREA); //0x0037
    
    //Write arguments
    it8951WriteData(usX);
    it8951WriteData(usY);
    it8951WriteData(usW);
    it8951WriteData(usH);
    it8951WriteData(usDpyMode);
	it8951WriteData((uint16_t)ulDpyBufAddr);       //Display Buffer Base address[15:0]
    it8951WriteData((uint16_t)(ulDpyBufAddr>>16)); //Display Buffer Base address[26:16]
}
//-----------------------------------------------------------
// Display 1bpp Flow
//-----------------------------------------------------------
void ePaperDisplay1bpp()
{
    IT8951AreaImgInfo stAreaImgInfo;
	IT8951LdImgInfo stLdImgInfo;
    
	gpio_set_level(PIN_INK_ON, 1);
    //Prepare image
    //memset(gpFrameBuf, 0xff, (ePaperInfo.usPanelW * ePaperInfo.usPanelH)/8);
	//for (uint16_t i=1; i<600;i+=2) memset(gpFrameBuf+100*i, 0x00, 100);
	memcpy(gpFrameBuf, font1, (fWidth1 * fHeight1)/8);
	
    //Check TCon is free ? Wait TCon Ready (optional)
    it8951WaitForDisplayReady();
     
    //Load Image and Display
    //Set Load Area
    stAreaImgInfo.usX      = 0;
    stAreaImgInfo.usY      = 0;
    stAreaImgInfo.usWidth  = ePaperInfo.usPanelW;
    stAreaImgInfo.usHeight = ePaperInfo.usPanelH;
	
    //Load Image from Host to IT8951 Image Buffer
    //Setting Load image information
    stLdImgInfo.ulStartFBAddr    = (uint32_t) gpFrameBuf;
    stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
    stLdImgInfo.usPixelFormat    = IT8951_8BPP; 
    stLdImgInfo.usRotate         = IT8951_ROTATE_0;
    stLdImgInfo.ulImgBufBaseAddr = gulImgBufAddr;
	
    //Set Load Area for 1bpp
    stAreaImgInfo.usX      = stAreaImgInfo.usX/8;
    stAreaImgInfo.usY      = stAreaImgInfo.usY;
    stAreaImgInfo.usWidth  = stAreaImgInfo.usWidth/8;
    stAreaImgInfo.usHeight = stAreaImgInfo.usHeight;
	
    //Load Image from Host to IT8951 Image Buffer
    ePaperHostAreaPackedPixelWrite(&stLdImgInfo, &stAreaImgInfo);
	
    //Display Area - (x,y,w,h) with mode 2 for Gray Scale
    //e.g. if we want to set b0(Background color) for Black-0x00 , Set b1(Foreground) for White-0xFF
	ePaperDisplayArea1bpp(0,0, ePaperInfo.usPanelW, ePaperInfo.usPanelH, 2, 0x00, 0xFF);
}

//-----------------------------------------------------------
// Display 1bpp Flow Area
//-----------------------------------------------------------
void ePaperDisplay1bppArea(uint16_t x, uint16_t y, uint16_t xSize, uint16_t ySize)
{
    IT8951AreaImgInfo stAreaImgInfo;
	IT8951LdImgInfo stLdImgInfo;
   
    //Check TCon is free ? Wait TCon Ready (optional)
    it8951WaitForDisplayReady();
     
    //Load Image and Display
    //Set Load Area
    stAreaImgInfo.usX      = x;
    stAreaImgInfo.usY      = y;
    stAreaImgInfo.usWidth  = xSize;
    stAreaImgInfo.usHeight = ySize;
	
    //Load Image from Host to IT8951 Image Buffer
    //Setting Load image information
    stLdImgInfo.ulStartFBAddr    = (uint32_t) gpFrameBuf;
    stLdImgInfo.usEndianType     = IT8951_LDIMG_L_ENDIAN;
    stLdImgInfo.usPixelFormat    = IT8951_8BPP; 
    stLdImgInfo.usRotate         = IT8951_ROTATE_0;
    stLdImgInfo.ulImgBufBaseAddr = gulImgBufAddr;
	
    //Set Load Area for 1bpp
    stAreaImgInfo.usX      = stAreaImgInfo.usX/8;
    stAreaImgInfo.usY      = stAreaImgInfo.usY;
    stAreaImgInfo.usWidth  = stAreaImgInfo.usWidth/8;
    stAreaImgInfo.usHeight = stAreaImgInfo.usHeight;
	
    //Load Image from Host to IT8951 Image Buffer
    ePaperHostAreaPackedPixelWriteNbr(&stLdImgInfo, &stAreaImgInfo);
	
    //Display Area - (x,y,w,h) with mode 2 for Gray Scale
    //e.g. if we want to set b0(Background color) for Black-0x00 , Set b1(Foreground) for White-0xFF
	ePaperDisplayArea1bpp(x, y, xSize, stAreaImgInfo.usHeight, 2, 0x00, 0xFF);
}

void ePaperText(uint16_t value, uint8_t msg)
{
	#define XPOS3 32
	#define XPOS2 XPOS3+160
	#define XPOS1 XPOS2+160
	#define YPOS  96
	#define DELTA_VALUE 3
	#define REFRESHTIME 12*10
	
	static uint16_t prvDigit[]={13,13,13};
	static uint16_t prevValue=255;
	static uint8_t refresh = 0;
	uint16_t yPos=YPOS;
	uint16_t digit;
	uint16_t xPos[]={XPOS1, XPOS2, XPOS3};
	
	if (msg==1)
	{
		gpio_set_level(PIN_INK_ON, 1);	
		gpio_set_level(PIN_INK_CS, 1);
		it8951Rst();
		vTaskDelay(1 / portTICK_RATE_MS);
		it8951SystemRun();
		vTaskDelay(10 / portTICK_RATE_MS);

		memcpy(gpFrameBuf, &font[11], (fWidth * fHeight)/8);
		ePaperDisplay1bppArea(xPos[0], yPos, fWidth, fHeight);
		
		digit = value/10;
		digit = value-(digit*10);
		memcpy(gpFrameBuf, &font[digit], (fWidth * fHeight)/8);
		ePaperDisplay1bppArea(xPos[1], yPos, fWidth, fHeight);
		
		memcpy(gpFrameBuf, &font[11], (fWidth * fHeight)/8);
		ePaperDisplay1bppArea(xPos[2], yPos, fWidth, fHeight);
		it8951Sleep();
		return;
	}
	else if (msg==2)
	{
		gpio_set_level(PIN_INK_ON, 1);	
		gpio_set_level(PIN_INK_CS, 1);
		it8951Rst();
		vTaskDelay(1 / portTICK_RATE_MS);
		it8951SystemRun();
		vTaskDelay(10 / portTICK_RATE_MS);
		
		memcpy(gpFrameBuf, &font[11], (fWidth * fHeight)/8);
		ePaperDisplay1bppArea(xPos[0], yPos, fWidth, fHeight);
		
		memcpy(gpFrameBuf, &font[11], (fWidth * fHeight)/8);
		ePaperDisplay1bppArea(xPos[1], yPos, fWidth, fHeight);
		
		memcpy(gpFrameBuf, &font[11], (fWidth * fHeight)/8);
		ePaperDisplay1bppArea(xPos[2], yPos, fWidth, fHeight);
		it8951Sleep();
		return;		
	}
	
	printf("%d, %d\n", value, prevValue);
	uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	
	//check X%
	if (value<10) value = 0;
	if (value>900) value = 900;

	if (value==prevValue) return;
	if (abs(value-prevValue)<=DELTA_VALUE) return;
	prevValue = value;	
	
	// update digit
	gpio_set_level(PIN_INK_ON, 1);	
	gpio_set_level(PIN_INK_CS, 1);
	vTaskDelay(20 / portTICK_RATE_MS);
	it8951Rst();
	vTaskDelay(5 / portTICK_RATE_MS);
	it8951SystemRun();
	printf("ePaper run\n");
	uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	
	if(refresh++>REFRESHTIME)
	{
		refresh=0;
		ePaperDisplay1bpp();
	}
	for(int i=0; i<3; i++)
	{
		digit = value/10;
		digit = value-(digit*10);
		value = value/10;
		if ((digit==0)&& (i!=0) && (value==0)) digit=10; 
		if (digit!=prvDigit[i])
		{
			// Prepare image
			memcpy(gpFrameBuf, &font[digit], (fWidth * fHeight)/8);
			// Transfert image
			ePaperDisplay1bppArea(xPos[i], yPos, fWidth, fHeight);
		}
		prvDigit[i] = digit;
	}
	
	it8951Sleep();
}
