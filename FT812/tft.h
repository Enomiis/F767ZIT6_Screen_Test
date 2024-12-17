/**
 * @file tft.h
 *
 */

#ifndef DISP_H
#define DISP_H

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
//#include "../lvgl/src/misc/lv_color.h"
//#include "../lvgl/src/misc/lv_area.h"

/*********************
 *      DEFINES
 *********************/
#define TFT_HOR_RES 800
#define TFT_VER_RES 480

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void tft_init(void);
void TFT_bitmap_display(void);
void TFT_cmd_memset(void);
void DMA2_Stream3_TransferComplete(void);

/**********************
 *      MACROS
 **********************/

#endif
