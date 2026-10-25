#ifndef TFT_H
#define TFT_H

#include "pico/stdlib.h"
#include "lcd.h"

static void init_display(void)
{
  // Settings for Waveshare RP2040-LCD0.96
  pico_st7735_80x160_config_t config = {
    SPI_CLK_FREQ_DEFAULT,
    spi1,
    PIN_LCD_SPI1_CS_WAVESHARE,
    PIN_LCD_SPI1_SCK_WAVESHARE,
    PIN_LCD_SPI1_MOSI_WAVESHARE,
    PIN_LCD_DC_WAVESHARE,
    PIN_LCD_RST_WAVESHARE,
    PIN_LCD_BLK_WAVESHARE,
    PWM_BLK_DEFAULT,
    INVERSION_DEFAULT,
    RGB_ORDER_DEFAULT,
    ROTATION_DEFAULT,
    H_OFS_DEFAULT,
    V_OFS_DEFAULT,
    X_MIRROR_DEFAULT
  };

  LCD_Config(&config);
  LCD_Init();
  LCD_Clear(BLACK);
}

static void clear_display(void)
{
  LCD_Clear(BLACK);
}

static void display_text(const char *text, const uint8_t line, const uint16_t color = WHITE)
{
  LCD_ShowString(0, line * 16, (const u8*)text, color);
}

#endif // TFT_H
