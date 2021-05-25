#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"

#define as1113_c

#include "AS1113.h"
#include "include.h"

uint16_t CMD_LED[NBR_LED+1]=
/*
{
	0x0000, 0x0100, 0x0300, 0x0700, 0x0f00, 0x1f00, 0x3f00, 0x7f00, 0xff00,
	0xff01, 0xff03, 0xff07, 0xff0f, 0xff1f, 0xff3f, 0xff7f, 0xffff
};*/
{
	0x0000, 0x0100, 0x0300, 0x0700, 0x0f00, 0x1f00, 0x3f00, 0x7f00, 0xff00
};
uint16_t CMD_LED_TEST[NBR_LED]=
{
	0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000
	//0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080
};

uint16_t panel[NBR_PANEL][NBR_LED]=
/*
{
	{37, 77, 117, 157, 197, 237, 277, 317, 357, 437, 477, 517, 557, 597, 630},
	{37, 77, 117, 157, 197, 237, 277, 317, 357, 437, 477, 517, 557, 597, 630},
	{37, 77, 117, 157, 197, 237, 277, 317, 357, 437, 477, 517, 557, 597, 630},
	{37, 77, 117, 157, 197, 237, 277, 317, 357, 437, 477, 517, 557, 597, 630},	
	{37, 77, 117, 157, 197, 237, 277, 317, 357, 437, 477, 517, 557, 597, 630},
	{37, 77, 117, 157, 197, 237, 277, 317, 357, 437, 477, 517, 557, 597, 630}			
};*/
{
	{37, 57, 97, 127, 157, 257, 517, 630},
	{37, 57, 97, 127, 157, 257, 517, 630},
	{37, 57, 97, 127, 157, 257, 517, 630},
	{37, 57, 97, 127, 157, 257, 517, 630},
	{37, 57, 97, 127, 157, 257, 517, 630},
	{37, 57, 97, 127, 157, 257, 517, 630},	
};

spi_device_handle_t spi;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;

//Initialize the LED driver
void led_init(spi_host_device_t host, uint16_t enableDMA)
{
	esp_err_t ret;

	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL<<PIN_LED_CS);
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	gpio_set_level(PIN_LED_CS, 0);
	
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL<<PIN_LED_OE);
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	gpio_set_level(PIN_LED_OE, 0);

    buscfg.miso_io_num=PIN_LED_MISO;
    buscfg.mosi_io_num=PIN_LED_MOSI;
    buscfg.sclk_io_num=PIN_LED_CLK;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;
    buscfg.max_transfer_sz=16+8;
 
    //Initialize the SPI bus
    ret=spi_bus_initialize(host, &buscfg, enableDMA);

    devcfg.clock_speed_hz=1000000,  
    devcfg.mode=0,                                 
    devcfg.spics_io_num=GPIO_NUM_NC ,               
    devcfg.queue_size=7,                           
	//Attach the LED driver to the SPI bus
    ret=spi_bus_add_device(host, &devcfg, &spi);
    ESP_ERROR_CHECK(ret); 
	 
	//gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_33);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

// set leds state
void led_set(uint16_t power, uint8_t PanelType, bool testMode)
{
    esp_err_t ret;
    spi_transaction_t t;
	char data[]={0x00,0x00};
	uint16_t * pData=(uint16_t *) data;
	
	gpio_set_level(PIN_LED_CS, 0);
	if(testMode)
	{	// test mode
		*pData=CMD_LED_TEST[power];
	}
	else
	{	// normal mode
		for(int i=0;i<NBR_LED;i++)
		{
			if (power < panel[PanelType][i])
			{
				*pData=CMD_LED[i];
				break;
			}			
		}
	}
    memset(&t, 0, sizeof(t));       
    t.length=2*8;                   
    t.tx_buffer=data;               
    ret=spi_device_polling_transmit(spi, &t);  
	vTaskDelay(1/portTICK_RATE_MS);
	gpio_set_level(PIN_LED_CS, 1);
	vTaskDelay(1/portTICK_RATE_MS);
	gpio_set_level(PIN_LED_CS, 0);
    assert(ret==ESP_OK);          
}

// set leds state
void led_test()
{
	for(int i=0; i<NBR_LED;i++)
	{
		led_set(i, 0, true);
		vTaskDelay(100/portTICK_RATE_MS);
	}	
	led_set(0, 0, false);
	vTaskDelay(1/portTICK_RATE_MS);	
}
