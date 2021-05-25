#ifndef INCLUDE_H_INCLUDED
#define INCLUDE_H_INCLUDED

#include "esp_system.h"
#include "IT8951.h"
#include "ePaper.h"
#include "AS1113.h"
#include "i2cLum.h"
#include "AsciiLib.h"
#include "BMPLib.h"

#define NBR_LED 8
#define NBR_PANEL 6

#define PANEL1980 			0
#define PANEL1990 			1
#define PANEL2010 			2
#define PANEL_WHITE 		3
#define PANEL_KALEO 		4
#define PANEL_TERRACOTA 	5

#define PANEL1980PW			157
#define PANEL1990PW			180
#define PANEL2010PW			325
#define PANEL_WHITEPW 		175
#define PANEL_KALEOPW 		121
#define PANEL_TERRACOTAPW	197

#ifdef evologia_main_c
#define MAIN_EXTERN
#else
#define MAIN_EXTERN extern
#endif

#ifdef as1113_c
#define AS_EXTERN
#else
#define AS_EXTERN extern
#endif

#ifdef it8951_c
#define IT_EXTERN
#else
#define IT_EXTERN extern
#endif

IT_EXTERN uint8_t *gpFrameBuf;
IT_EXTERN uint32_t gulImgBufAddr;
IT_EXTERN I80IT8951DevInfo ePaperInfo;

#ifdef it8951_c
#define IT_EXTERN
#else
#define IT_EXTERN extern
#endif

#ifdef asciilib_c
#define ASCIILIB_EXTERN
#else
#define ASCIILIB_EXTERN extern
#endif

ASCIILIB_EXTERN const uint8_t font[12][fHeight*fWidth/8];

#ifdef bmplib_c
#define BMPLIB_EXTERN
#else
#define BMPLIB_EXTERN extern
#endif

BMPLIB_EXTERN const uint8_t font1[fHeight1*fWidth1/8];

#endif //INCLUDE_H_INCLUDED
