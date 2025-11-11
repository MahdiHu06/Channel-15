#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "lcd.h"
#include <stdio.h>
#include <views.h>

#define PIN_SDI    15
#define PIN_CS     13
#define PIN_SCK    14
#define PIN_DC     16
#define PIN_nRESET 17

void init_spi_lcd() {
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    gpio_set_function(PIN_DC, GPIO_FUNC_SIO);
    gpio_set_function(PIN_nRESET, GPIO_FUNC_SIO);

    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_set_dir(PIN_nRESET, GPIO_OUT);

    gpio_put(PIN_CS, 1); // CS high
    gpio_put(PIN_DC, 0); // DC low
    gpio_put(PIN_nRESET, 1); // nRESET high

    // initialize SPI1 with 48 MHz clock
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SDI, GPIO_FUNC_SPI);
    spi_init(spi1, 100 * 1000 * 1000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

int main() {
    stdio_init_all();

    keypad_init_pins();
    keypad_init_timer();

    init_spi_lcd();

    LCD_Setup();
    LCD_Clear(0x0000);

    typedef enum {
        STATE_MENU,
        STATE_LIVE_DATA,
        STATE_HISTORICAL_DATA,
        STATE_SETTINGS,
        STATE_ABOUT
    } State;

    State currentState = STATE_MENU;

    uint8_t highlight = 0;
    menu(highlight);

    while (true) {
        uint16_t keyevent = key_pop();

        switch (currentState) {
        case STATE_MENU:
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '2') {
                    highlight = (highlight + 3) % 4;
                    menu(highlight);
                } else if (key == '8') {
                    highlight = (highlight + 1) % 4;
                    menu(highlight);
                } else if (key == '5') {
                    switch (highlight) {
                    case 0:
                        currentState = STATE_LIVE_DATA;
                        LCD_Clear(LGRAY);
                        break;
                    case 1:
                        currentState = STATE_HISTORICAL_DATA;
                        LCD_Clear(GREEN);
                        break;
                    case 2:
                        currentState = STATE_SETTINGS;
                        LCD_Clear(MAGENTA);
                        break;
                    case 3:
                        currentState = STATE_ABOUT;
                        LCD_Clear(BLUE);
                        break;
                    }
                }
            }
            break;
        
        case STATE_LIVE_DATA:
            // TODO: Implement live data display
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(highlight);
                }
            }
            break;

        case STATE_HISTORICAL_DATA:
            // TODO: Implement historical data display
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(highlight);
                }
            }
            break;

        case STATE_SETTINGS:
            // TODO: Implement settings display
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(highlight);
                }
            }
            break;

        case STATE_ABOUT:
            // TODO: Implement about display
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(highlight);
                }
            }
            break;
    
        default:
            break;
        }
        sleep_us(1000);
    }
    
    return 0;
}