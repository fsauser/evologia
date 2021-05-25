#ifndef AS1113_H_INCLUDED
#define AS1113_H_INCLUDED

#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

// Driver LED AS1113
#define PIN_LED_MISO GPIO_NUM_37
#define PIN_LED_MOSI GPIO_NUM_25
#define PIN_LED_CLK  GPIO_NUM_26
#define PIN_LED_CS   GPIO_NUM_14
#define PIN_LED_OE   GPIO_NUM_32

void led_init(spi_host_device_t host, uint16_t enableDMA);
void led_set(uint16_t power, uint8_t PanelType, bool testMode);
void led_test();


#endif // AS1113_H_INCLUDED