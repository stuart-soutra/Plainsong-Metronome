/*
 * oled.h
 *
 *  Created on: Jun 12, 2025
 *      Author: 40015802
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "stm32f4xx_hal.h" // Change to match your STM32 series

// External I2C handle (e.g. from main.c)
extern I2C_HandleTypeDef hi2c1;

#define OLED_I2C_ADDR 0x3C << 1 // 0x3C is common SSD1309 addr (7-bit, shifted for HAL)

void OLED_Init(void);
void OLED_Clear(void);
void OLED_UpdateScreen(void);
void OLED_DrawBitmap(const uint8_t* bitmap);
void OLED_DisplayStartupLogo(void);
void OLED_ShowDefaultScreen(uint8_t bpm, uint8_t volume, int is_playing);
void OLED_ShowSettingsScreen(void);
void OLED_ShowSamplesScreen(void);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_Print(const char *str);
void OLED_Reset(void);

#endif /* INC_OLED_H_ */
