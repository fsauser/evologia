/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/rom/uart.h"
#include "soc/rtc_wdt.h"
#include "driver/i2c.h"

#define evologia_main_c
#include "include.h"

// switch 
#define PIN_SW1 P2
#define PIN_SW2 P12
#define PIN_SW4_S0   GPIO_NUM_38
#define PIN_SW4_S1   GPIO_NUM_35
#define PIN_SW4_S2   GPIO_NUM_34

uint8_t readSW4();
void initGPIO();

void app_main()
{
    uint8_t error=0;;
	uint16_t pwr=0;
	uint8_t panel;
	
	vTaskDelay(100 / portTICK_RATE_MS);
    //Initialize the LED driver
    led_init(VSPI_HOST, 0); 
	led_test();
	printf("Power on LED\n");
	uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	vTaskDelay(10 / portTICK_RATE_MS);
	//Initialize the ePaper driver
    it8951_init(HSPI_HOST, 0);
	
	// Read switch 
	initGPIO();
	panel = readSW4();
	printf("panel %d\n",panel);

	ePaperGetInfo();
	printf("ePaper on\n");
	uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	rtc_wdt_feed();
	ePaperDisplay1bpp();
	rtc_wdt_feed();
	printf("Display\n");
	uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	ePaperText(panel, 1);
	
	VEML7700_init();
	printf("VEML on\n");
	uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	
	// Wake up in 4 seconds, or when button is pressed 
	esp_sleep_enable_timer_wakeup(4000000);
	//esp_light_sleep_start();
	
    for(;;)
	{
		// Enter sleep mode 
		printf("Enter sleep mode\n");
	    uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
        esp_light_sleep_start();
		rtc_wdt_feed();
		// Leave sleep mode 
		pwr=VEML7700_getValue(panel);
		rtc_wdt_feed();
		
		if ((pwr==11111)&&(error==0))
		{
			printf("Error %d\n", pwr);
			pwr=0;
			uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
			ePaperText(0, 2);
			error=1;
		}
		else if (error==0) ePaperText(pwr, 0);
		else if ((error==1)&&(pwr==0))
		{
			esp_restart();
		}
		
		//printf("Power = %d W\n", pwr);
	    //uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
		led_set(pwr, panel, false);
		rtc_wdt_feed();
	}
}

void initGPIO()
{
	// SW4
	static gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_SW4_S0);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_SW4_S1);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<PIN_SW4_S2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_5);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_27);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_19);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_0);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	gpio_set_level(GPIO_NUM_0, 0);
}

uint8_t readSW4()
{
	uint8_t ulData = gpio_get_level(PIN_SW4_S2);
	ulData = ulData<<1;
	ulData = ulData | gpio_get_level(PIN_SW4_S1);
	ulData = ulData<<1;
	ulData = ulData | gpio_get_level(PIN_SW4_S0);
	return ulData;
}
