#include "oled.h"
#include "oled_font.h"  // Include your own 5x7 or 8x8 font data
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)

static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];

static uint8_t cursor_x = 0;
static uint8_t cursor_y = 0;



void OLED_Print(const char *str) {
    while (*str) {
        char c = *str++;
        if (c < 32 || c > 126) c = ' '; // fallback for unknown chars
        for (int i = 0; i < 6; i++) {
        	uint8_t line = oled_font6x8[(uint8_t)c - 32][i];
            uint16_t index = cursor_x + (cursor_y * 128);
            if (index < sizeof(oled_buffer)) {
                oled_buffer[index] = line;
                cursor_x++;
            }
        }
    }
}

void OLED_SetCursor(uint8_t x, uint8_t y) {
    cursor_x = x;
    cursor_y = y;
}

static void OLED_WriteCommand(uint8_t cmd) {
    uint8_t d[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, d, 2, HAL_MAX_DELAY);
}

static void OLED_WriteData(uint8_t* data, size_t size) {
    uint8_t* buf = malloc(size + 1);
    buf[0] = 0x40;
    memcpy(&buf[1], data, size);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, buf, size + 1, HAL_MAX_DELAY);
    free(buf);
}

void OLED_Reset(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_Delay(10);
}

void OLED_Init(void) {
	OLED_Reset();

    OLED_WriteCommand(0xAE); // Display off
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F); // MUX ratio
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00); // Display offset
    OLED_WriteCommand(0x40); // Start line = 0
    OLED_WriteCommand(0xA1); // Segment remap
    OLED_WriteCommand(0xC8); // COM output scan direction
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12); // COM pins
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0x7F); // Contrast
    OLED_WriteCommand(0xA4); // Resume RAM content display
    OLED_WriteCommand(0xA6); // Normal display
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80); // Clock
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x10); // DISable charge pump
    OLED_WriteCommand(0xAF); // Display ON
}

void OLED_Clear(void) {
    memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

void OLED_UpdateScreen(void) {
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        OLED_WriteCommand(0xB0 + page);
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);
        OLED_WriteData(&oled_buffer[OLED_WIDTH * page], OLED_WIDTH);
    }
}

void OLED_DrawBitmap(const uint8_t* bitmap) {
    memcpy(oled_buffer, bitmap, sizeof(oled_buffer));
    OLED_UpdateScreen();
}

void OLED_DisplayStartupLogo(void) {
    extern const uint8_t company_logo[]; // Declare your logo byte array in a separate C file
    OLED_DrawBitmap(company_logo);
    HAL_Delay(3000);
}

void OLED_ShowDefaultScreen(uint8_t bpm, uint8_t volume, int is_playing) {
    OLED_Clear();
    // Draw BPM
    OLED_SetCursor(0, 0);
    char buf[16];
    sprintf(buf, "VOL: %d", volume);
    OLED_Print(buf);

    // Draw Volume
    OLED_SetCursor(70, 0);
    sprintf(buf, "BPM: %d", bpm);
    OLED_Print(buf);

    // Playing/Paused
    OLED_SetCursor(30, 7);
    if (is_playing)
        OLED_Print("** PLAYING **");
    else
        OLED_Print(".. paused ..");

    OLED_UpdateScreen();
}

void OLED_ShowSettingsScreen(void) {
    OLED_Clear();                         // Clear previous content
    OLED_SetCursor(10, 3);                // Set cursor to middle row (y=3), a bit from the left (x=10)
    OLED_Print("Settings Screen");        // Print the label
    OLED_UpdateScreen();                  // Push buffer to display
}

void OLED_ShowSamplesScreen(void) {
    OLED_Clear();

    OLED_SetCursor(20, 0);  // X = 20, Page = 0 (top)
    OLED_Print("Sample Select");

    OLED_UpdateScreen();
}


