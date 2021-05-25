#ifndef I2CLUM_H_INCLUDED
#define I2CLUM_H_INCLUDED

#include "esp_system.h"
#include "driver/i2c.h"

void VEML7700_init();
uint32_t VEML7700_getValue(uint8_t);

#endif // I2CLUM_H_INCLUDED