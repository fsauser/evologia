#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/uart.h"

#define it8951_c

#include "include.h"

static spi_device_handle_t spi1;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;

//-----------------------------------------------------------------
void swapByte(uint8_t* dst, uint8_t* src, uint16_t nbr)
{
	uint16_t i;
	
	for(i=0;i<nbr;i+=2)
	{
		dst[i]=src[i+1];
		dst[i+1]=src[i];
	}
}
//-----------------------------------------------------------------
void delayNop()
{
	uint32_t i;
	for (i=0;i<1000;i++);	
}
//-----------------------------------------------------------------
void it8951_cmd(const uint8_t cmd)
{
    esp_err_t ret;
	
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi1, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//-----------------------------------------------------------------
void it8951WaitForReady()
{
	uint8_t ulData = gpio_get_level(PIN_INK_RDY);
	
	while(ulData == 0)
	{
		 ulData = gpio_get_level(PIN_INK_RDY);
	}
}
//-----------------------------------------------------------------
void it8951Send(uint8_t *data, uint32_t nbr)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=nbr*8;                //Command is 16 bits
    t.tx_buffer=data;              //The data is the cmd itself
    ret=spi_device_polling_transmit(spi1, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//-----------------------------------------------------------------
void it8951Receive(uint8_t *data, uint8_t nbrByte)
{
    spi_transaction_t t;
	
    memset(&t, 0, sizeof(t));
	t.rxlength=8*nbrByte;
	t.rx_buffer = data;
    esp_err_t ret = spi_device_polling_transmit(spi1, &t);
    assert( ret == ESP_OK );
}

//Host controller function 3---Write Data to host data Bus
//-----------------------------------------------------------
void it8951WriteData(uint16_t usData)
{
	//Set Preamble for Write Data
	uint8_t wPreamble[] = {0x00,0x00}; 
	uint8_t *ptr=(uint8_t*)&usData; 

	it8951WaitForReady();
	gpio_set_level(PIN_INK_CS, 0);
	delayNop();
	it8951Send(wPreamble, 2);
	it8951WaitForReady();
	wPreamble[0]=*(ptr+1); 
	wPreamble[1]=*ptr; 
	it8951Send(wPreamble, 2);
	gpio_set_level(PIN_INK_CS, 1);
	delayNop();

}
//Host controller function 3---Write Data to host data Bus
//-----------------------------------------------------------
void it8951WriteNData(uint8_t * usData, uint32_t nbr)
{
	//Set Preamble for Write Data
	uint8_t wPreamble[] = {0x00,0x00}; 
	
	it8951WaitForReady();
	gpio_set_level(PIN_INK_CS, 0);
	delayNop();
	it8951Send(wPreamble, 2);
	it8951WaitForReady(); 
	it8951Send(usData, nbr);
	gpio_set_level(PIN_INK_CS, 1);
	delayNop();

}
//-----------------------------------------------------------------
void it8951WriteCmdCode(uint16_t usCmdCode)
{
	uint8_t wPreamble[] = {0x60,0x00}; 
	uint8_t *ptr=(uint8_t*)&usCmdCode; 

	//SPI_Enable;
	it8951WaitForReady();
	gpio_set_level(PIN_INK_CS, 0);
	delayNop();
	it8951Send(wPreamble, 2);	
	//printf("%02x%02x -", wPreamble[0], wPreamble[1]);
	//uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	it8951WaitForReady();
	wPreamble[0]=*(ptr+1); 
	wPreamble[1]=*ptr; 
	it8951Send(wPreamble, 2);	
	//printf(" %02x%02x\n", wPreamble[0], wPreamble[1]);
	//uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	gpio_set_level(PIN_INK_CS, 1);
	delayNop();	
	//SPI_Disable;
}
//-----------------------------------------------------------
//Host controller function 4---Read Data from host data Bus
//-----------------------------------------------------------
uint16_t it8951ReadData()
{
	uint16_t wRData; 
	uint8_t wPreamble[] = {0x10,0x00}; 

	it8951WaitForReady();
	gpio_set_level(PIN_INK_CS, 0);
	delayNop();
	it8951Send(wPreamble, 2);
	it8951WaitForReady();
	it8951Receive(wPreamble, 2);
	it8951WaitForReady();
	it8951Receive(wPreamble, 2);		
	gpio_set_level(PIN_INK_CS, 1);
	delayNop();
	swapByte((uint8_t*)&wRData, wPreamble, 2);
		
	return wRData;
}

//-----------------------------------------------------------
//  Read Burst N words Data
//-----------------------------------------------------------
void it8951ReadNData(uint8_t* pwBuf, uint16_t ulSizeCnt)
{
	uint8_t wPreamble[] = {0x10, 0x00};

	it8951WaitForReady();
	gpio_set_level(PIN_INK_CS, 0);
	delayNop();
	it8951Send(wPreamble, 2);
	it8951WaitForReady();
	it8951Receive(pwBuf, 2);
	it8951WaitForReady();
	it8951Receive(pwBuf, ulSizeCnt);
	gpio_set_level(PIN_INK_CS, 1); 
}

//Initialize the IT8951 driver
void it8951_init(spi_host_device_t host, uint16_t enableDMA)
{
	esp_err_t ret;
	
	//Initialize non-SPI GPIOs
	// Reset actif to '0'
	static gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_INK_RST);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	gpio_set_level(PIN_INK_RST, 1);
	
	// CS actif to '1'
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_INK_CS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
	gpio_set_level(PIN_INK_CS, 0);
	
	// RDY actif to '1'
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_INK_RDY);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	gpio_get_level(PIN_INK_RDY);
	
	// ePaper ON actif to '1'
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_INK_ON);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	gpio_set_level(PIN_INK_ON, 1);

	//Initialize the SPI PORT
    buscfg.miso_io_num=PIN_INK_MISO;
    buscfg.mosi_io_num=PIN_INK_MOSI;
    buscfg.sclk_io_num=PIN_INK_CLK;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;
    buscfg.max_transfer_sz=SPI_MAX_DMA_LEN;
 
    //Initialize the SPI bus
    ret=spi_bus_initialize(host, &buscfg, enableDMA);
	devcfg.command_bits=0;           	
	devcfg.address_bits=0;
	devcfg.dummy_bits=0;
    devcfg.clock_speed_hz=2*1000*1000,  //Clock out at 2MHz (max 24MHz pour IT8951)
	devcfg.duty_cycle_pos=0;       
	devcfg.mode=0;
	devcfg.spics_io_num=-1;
	devcfg.cs_ena_posttrans=0;        	
	devcfg.cs_ena_pretrans=0;        
	devcfg.queue_size=7;
	devcfg.flags = SPI_DEVICE_HALFDUPLEX;

	//Attach the IT8951 driver to the SPI bus
    ret=spi_bus_add_device(host, &devcfg, &spi1);
    ESP_ERROR_CHECK(ret);
	
	//Reset the display
	gpio_set_level(PIN_INK_CS, 0);
    gpio_set_level(PIN_INK_RST, 0);
    vTaskDelay(10/portTICK_RATE_MS);
    gpio_set_level(PIN_INK_RST, 1);
}

void ePaperGetInfo()
{
	uint8_t data[64];
	uint8_t nbrByte=sizeof(ePaperInfo);
	
	it8951WriteCmdCode(USDEF_I80_CMD_GET_DEV_INFO);
    it8951ReadNData(data, nbrByte);
	swapByte((uint8_t *)&ePaperInfo, data, nbrByte);

    printf("\nePaper info\n");
    printf("------------------------------\n");
    printf("width : %d\n", ePaperInfo.usPanelW);  
    printf("height : %d\n", ePaperInfo.usPanelH);
    printf("addrL : %d\n", ePaperInfo.usImgBufAddrL);
    printf("addrH : %d\n", ePaperInfo.usImgBufAddrH);
	printf("FW ver. : %s\n", (char *)ePaperInfo.usFWVersion);
    printf("LUT ver. : %s\n", (char *)ePaperInfo.usLUTVersion);
    printf("------------------------------\n");
    printf("\n");
	
	//Set to Enable I80 Packed mode
 	it8951WriteReg(I80CPCR, 0x0001);
	
	printf("VCOM = -%.02fV\n",(float)it8951GetVCOM()/1000);
	int size = ePaperInfo.usPanelW * ePaperInfo.usPanelH/8;
	gpFrameBuf = (uint8_t*) heap_caps_malloc(size * sizeof(uint8_t), MALLOC_CAP_DMA);
	//gpFrameBuf = heap_caps_malloc(ePaperInfo.usPanelW * ePaperInfo.usPanelH, MALLOC_CAP_8BIT)
	assert( gpFrameBuf != NULL );
	//ESP_ERROR_CHECK(gpFrameBuf);
	
 	gulImgBufAddr = ePaperInfo.usImgBufAddrL | (ePaperInfo.usImgBufAddrH << 16);

/*
	if (1.72 != it8951GetVCOM())
	{
		it8951SetVCOM(VCOM);
		printf("VCOM = -%.02fV\n",(float)it8951GetVCOM()/1000);
	}*/
}
//-----------------------------------------------------------
// SYS_RUN
//-----------------------------------------------------------
void it8951SystemRun()
{
    it8951WriteCmdCode(IT8951_TCON_SYS_RUN);
}
//-----------------------------------------------------------
// Write command to host data Bus with aruments
//-----------------------------------------------------------
void it8951SendCmdArg(uint16_t usCmdCode,uint16_t* pArg, uint16_t usNumArg)
{
	uint8_t data[32];
    //Send Cmd code
    it8951WriteCmdCode(usCmdCode);
    //Send Data
	swapByte(data, (uint8_t*)pArg, usNumArg*2); 
 
	it8951WriteNData(data, usNumArg*2);
}
//-----------------------------------------------------------
// STANDBY
//-----------------------------------------------------------
void it8951StandBy()
{
    it8951WriteCmdCode(IT8951_TCON_STANDBY);
}

//-----------------------------------------------------------
// SLEEP
//-----------------------------------------------------------
void it8951Sleep()
{
    it8951WriteCmdCode(IT8951_TCON_SLEEP);
	vTaskDelay(1 / portTICK_RATE_MS);
	gpio_set_level(PIN_INK_ON, 0);
	gpio_set_level(PIN_INK_CS, 0);
	gpio_set_level(PIN_INK_RST, 0);
	//gpio_set_level(PIN_INK_RST, 0);
	//gpio_set_level(PIN_INK_MOSI, 0);
	//gpio_set_level(PIN_INK_CLK, 0);

}
//-----------------------------------------------------------
// REG_RD
//-----------------------------------------------------------
uint16_t it8951ReadReg(uint16_t usRegAddr)
{
	uint16_t usData;
	
	//Send Cmd and Register Address
	it8951WriteCmdCode(IT8951_TCON_REG_RD);
	it8951WriteData(usRegAddr);
	//Read data from Host Data bus
	usData = it8951ReadData();
	return usData;
}
//-----------------------------------------------------------
// REG_WR
//-----------------------------------------------------------
void it8951WriteReg(uint16_t usRegAddr,uint16_t usValue)
{
	uint8_t data[4];
	//Send Cmd , Register Address and Write Value
	it8951WriteCmdCode(IT8951_TCON_REG_WR);
    //Send Data
	swapByte(&data[0], (uint8_t*)&usRegAddr, 2); 
	swapByte(&data[2], (uint8_t*)&usValue, 2); 

	it8951WriteNData(data, 4);
}

uint16_t it8951GetVCOM(void)
{
	uint16_t vcom;
	
	it8951WriteCmdCode(USDEF_I80_CMD_VCOM);
	it8951WriteData(0);
	vcom = it8951ReadData();
	return vcom;
}

void it8951SetVCOM(uint16_t vcom)
{
	uint16_t val=1;
	uint8_t data[4];
	it8951WriteCmdCode(USDEF_I80_CMD_VCOM);
	//Send Data
	swapByte(&data[0], (uint8_t*)&val, 2); 
	swapByte(&data[2], (uint8_t*)&vcom, 2); 

	it8951WriteNData(data, 4);
}
//-----------------------------------------------------------
// Wait for LUT Engine Finish
// Polling Display Engine Ready by LUTNo
//-----------------------------------------------------------
void it8951WaitForDisplayReady()
{
	//Check IT8951 Register LUTAFSR => NonZero Busy, 0 - Free
	while(it8951ReadReg(LUTAFSR));
}
//-----------------------------------------------------------
// Set Image buffer base address
//-----------------------------------------------------------
void it8951SetImgBufBaseAddr(uint32_t ulImgBufAddr)
{
	uint16_t usWordH = (uint16_t)((ulImgBufAddr >> 16) & 0x0000FFFF);
	uint16_t usWordL = (uint16_t)( ulImgBufAddr & 0x0000FFFF);
	//Write LISAR Reg
	it8951WriteReg(LISAR + 2 ,usWordH);
	it8951WriteReg(LISAR ,usWordL);
}
//-----------------------------------------------------------
// LD_IMG_AREA
//-----------------------------------------------------------
void it8951LoadImgAreaStart(IT8951LdImgInfo* pstLdImgInfo ,IT8951AreaImgInfo* pstAreaImgInfo)
{
    uint16_t usArg[5];
    //Setting Argument for Load image start
    usArg[0] = (pstLdImgInfo->usEndianType << 8 )
    |(pstLdImgInfo->usPixelFormat << 4)
    |(pstLdImgInfo->usRotate);
    usArg[1] = pstAreaImgInfo->usX;
    usArg[2] = pstAreaImgInfo->usY;
    usArg[3] = pstAreaImgInfo->usWidth;
    usArg[4] = pstAreaImgInfo->usHeight;
    //Send Cmd and Args
    it8951SendCmdArg(IT8951_TCON_LD_IMG_AREA , usArg , 5);
}
//-----------------------------------------------------------
// LD_IMG_END
//-----------------------------------------------------------
void it8951LoadImgEnd(void)
{
    it8951WriteCmdCode(IT8951_TCON_LD_IMG_END);
}

void it8951Rst()
{
	//Reset the display
	gpio_set_level(PIN_INK_CS, 0);
    gpio_set_level(PIN_INK_RST, 0);
    vTaskDelay(10/portTICK_RATE_MS);
    gpio_set_level(PIN_INK_RST, 1);
}


