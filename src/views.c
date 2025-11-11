#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "lcd.h"
#include <stdio.h>

void menu(uint8_t highlight) {
    LCD_Clear(0x0000);

    const char* options[] = {"Live Data", "Historical Data", "Settings", "About"};
    for (uint8_t i = 0; i < 4; i++) {
        if (i == highlight) {
            LCD_DrawFillRectangle(0, 30 + i * 40, 270, 60 + i * 40, 0x01CF);
            LCD_DrawString(10, 40 + i * 40, 0xFFFF, 0x01CF, options[i], 16, 1);
        } else {
            LCD_DrawString(10, 40 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);
        }
    }
}