/**
 * @file tft.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "../lv_conf.h"
#include "../lvgl/lvgl.h"
#include <string.h>
#include <stdlib.h>

#include "tft.h"
#include "../Riverdi_EVE/host_layer/platform.h"
#include "../Riverdi_EVE/app_layer/App_Common.h"

/*********************
 *      DEFINES
 *********************/

#if LV_COLOR_DEPTH != 16 && LV_COLOR_DEPTH != 24 && LV_COLOR_DEPTH != 32
#error LV_COLOR_DEPTH must be 16, 24, or 32
#endif

/**
  * @brief  LCD status structure definition
  */
#define LCD_OK                          ((uint8_t)0x00)
#define LCD_ERROR                       ((uint8_t)0x01)
#define LCD_TIMEOUT                     ((uint8_t)0x02)

#define SCREEN_BITMAP_ADDR	     0x00000000
#define EVE_RGB565               7UL
#define SCREEN_BUFFER_SIZE       (TFT_HOR_RES * TFT_VER_RES * 2)
#define SCREEN_DEFAULT_COLOR     0x000000UL
#define BYTES_PER_LINE           (TFT_HOR_RES * 2)
#define BYTES_PER_PIXEL          (2)

/**********************
 *      TYPEDEFS
 **********************/
volatile uint32_t flag_DMA_CH3_bsy;

void Send_DMA_Data16(uint8_t* buff, uint16_t dataSize)
{
    //LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
     
    LL_SPI_Disable(SPI1);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
    LL_DMA_ClearFlag_TC3(DMA2);
    LL_DMA_ClearFlag_TE3(DMA2);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_3);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_3);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, dataSize);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_3, (uint32_t)buff, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_3));
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
    LL_SPI_Enable(SPI1);
    while (!flag_DMA_CH3_bsy) {
    }
    flag_DMA_CH3_bsy = 0;
   
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
    LL_SPI_Disable(SPI1);
    LL_DMA_ClearFlag_TC3(DMA2);
    LL_DMA_ClearFlag_TE3(DMA2);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_3);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_3);
    LL_SPI_Enable(SPI1);
    //LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
}

void DMA2_Stream3_TransferComplete(void)
{
  flag_DMA_CH3_bsy = 1;
}

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*These 3 functions are needed by LittlevGL*/
//static void ex_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t * color_p);
static void tft_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t * color_p);

/**********************
 *  STATIC VARIABLES
 **********************/
static Gpu_Hal_Context_t host, *phost;
static lv_disp_drv_t disp_drv;

#if LV_COLOR_DEPTH == 16
typedef uint16_t uintpixel_t;
#elif LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32
typedef uint32_t uintpixel_t;
#endif

static lv_disp_t *our_disp = NULL;

/**********************
 *      MACROS
 **********************/

/**
 * Initialize your display here
 */

void tft_init(void)
{
  // LCD Initialization
  phost = &host;
  // Init HW Hal
  App_Common_Init(&host);
  // Setup memory in GRAM for fullscreen bitmap
  TFT_cmd_memset();
  SEGGER_RTT_printf(0U, "MEMSET OK\r\n");
  App_Flush_Co_Buffer(phost);
	Gpu_Hal_WaitCmdfifo_empty(phost);
  SEGGER_RTT_printf(0U, "CMD FIFO EMPTY\r\n");
  // Display bitmap using content from GRAM
  TFT_bitmap_display();
  SEGGER_RTT_printf(0U, "BITMAP DISPLAY OK\r\n");

  /*-----------------------------
	* Create a buffer for drawing
	*----------------------------*/

   /* LittlevGL requires a buffer where it draws the objects. The buffer's has to be greater than 1 display row*/

	static lv_disp_draw_buf_t disp_buf_1;
	static lv_color_t buf1_1[TFT_HOR_RES*10];
  static lv_color_t buf1_2[TFT_HOR_RES*10];
	lv_disp_draw_buf_init(&disp_buf_1, buf1_1, buf1_2, TFT_HOR_RES*10);   /*Initialize the display buffer*/


	/*-----------------------------------
	* Register the display in LittlevGL
	*----------------------------------*/

	lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

	/*Set up the functions to access to your display*/

	/*Set the resolution of the display*/
	disp_drv.hor_res = 800;
	disp_drv.ver_res = 480;

	/*Used to copy the buffer's content to the display*/
	disp_drv.flush_cb = tft_disp_flush;

	/*Set a display buffer*/
	disp_drv.draw_buf = &disp_buf_1;

	/*Finally register the driver*/
	our_disp = lv_disp_drv_register(&disp_drv);
}

void TFT_bitmap_display() {
  // Start display list
  Gpu_CoCmd_Dlstart(phost);
  // Clear screen
  App_WrCoCmd_Buffer(phost, CLEAR_COLOR_RGB(0, 0, 0));
  App_WrCoCmd_Buffer(phost, CLEAR(1, 1, 1));
  // Set up bitmap for screen size 800x480 px
  // Bitmap data is store in GRAM address 0
  App_WrCoCmd_Buffer(phost, BITMAP_SOURCE(SCREEN_BITMAP_ADDR));
  App_WrCoCmd_Buffer(phost, BITMAP_LAYOUT(RGB565, 576, 480));
  App_WrCoCmd_Buffer(phost, BITMAP_LAYOUT_H(1, 0));
  App_WrCoCmd_Buffer(phost, BITMAP_SIZE(NEAREST, BORDER, BORDER, 288, 480));
  App_WrCoCmd_Buffer(phost, BITMAP_SIZE_H(1, 0));
  App_WrCoCmd_Buffer(phost, BEGIN(BITMAPS));
  // Set display coordinate to (0, 0)
  App_WrCoCmd_Buffer(phost, VERTEX2II(0, 0, 0, 0));
  // Display bitmap
  App_WrCoCmd_Buffer(phost, END());
  App_WrCoCmd_Buffer(phost, DISPLAY());
  Gpu_CoCmd_Swap(phost);
  App_Flush_Co_Buffer(phost);
}

void TFT_cmd_memset() {
  // Set backgroud color to default in LVGL.
  //lv_color_t bg_color = LV_COLOR_MAKE(0xF5, 0xF5, 0xF5);
  lv_color_t bg_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
  Gpu_Hal_StartTransfer(phost, GPU_WRITE, SCREEN_BITMAP_ADDR);
  for (uint32_t i = 0; i < SCREEN_BUFFER_SIZE; i = i+2) {
    User_Gpu_Hal_Transfer16(phost, lv_color_to16(bg_color));
  }
  Gpu_Hal_EndTransfer(phost);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_flush_ready()' has to be called when finished
 * This function is required only when LV_VDB_SIZE != 0 in lv_conf.h*/
static void tft_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t * color_p)
{
  int32_t x1 = area->x1;
  int32_t x2 = area->x2;
  int32_t y1 = area->y1;
  int32_t y2 = area->y2;

  // Return if the area is out the screen
  if(x2 < 0) return;
  if(y2 < 0) return;
  if(x1 > TFT_HOR_RES - 1) return;
  if(y1 > TFT_VER_RES - 1) return;

  // Truncate the area to the screen
  int32_t act_x1 = x1 < 0 ? 0 : x1;
  int32_t act_y1 = y1 < 0 ? 0 : y1;
  int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
  int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

  SEGGER_RTT_printf(0U, "redrawn area is %dx%d\r\n", lv_area_get_width(area), lv_area_get_height(area));

  if ((act_x2 - act_x1) + 1 == TFT_HOR_RES) {
    Gpu_Hal_StartTransfer(phost, GPU_WRITE, SCREEN_BITMAP_ADDR + (act_y1*1600) + (2*act_x1));
    uint32_t data_size = ((act_x2 - act_x1 + 1) * (act_y2 - act_y1 + 1)) * 2;
    Send_DMA_Data16((void*)color_p, data_size);
    Gpu_Hal_EndTransfer(phost);
  } else {
    volatile uint16_t color_offset = 0;
    for (int32_t y = act_y1; y <= act_y2; y++) {
      Gpu_Hal_StartTransfer(phost, GPU_WRITE, SCREEN_BITMAP_ADDR + (y*1600) + (2*act_x1));
      uint32_t data_size = ((act_x2 - act_x1 + 1)) * 2;
      Send_DMA_Data16((void*)(color_p + (color_offset * (act_x2 - act_x1 + 1))), data_size);
      Gpu_Hal_EndTransfer(phost);
      color_offset++;
    }
  }

  lv_disp_flush_ready(drv);
}

