#ifndef EPAPER_H_INCLUDED
#define EPAPER_H_INCLUDED

#include "esp_system.h"


void ePaperDisplay1bpp();
void ePaperText(uint16_t value, uint8_t msg);

#endif // EPAPER_H_INCLUDED