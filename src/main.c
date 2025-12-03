#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "../include/lcd.h"
#include <stdio.h>
#include "../include/views.h"
#include "../include/radio.h"
#include "../include/speaker.h"
#include "speaker.c"

// TODO: Confirm pin numbers
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
    // Init Pins for Display
    init_spi_lcd();

    // Config Display
    LCD_Setup();
    LCD_Clear(0x0000);

    // Init Pins for Keypad
    keypad_init_pins();
    keypad_init_timer();

    stdio_init_all();

    // Init Pins for Radio
    initRadio(RADIO_SPI_SCK_PIN, RADIO_SPI_MISO_RX_PIN, RADIO_SPI_MOSI_TX_PIN, RADIO_SPI_CSN_PIN, RADIO_SPI_RESET_PIN);
    // Config Radio
    configRadio(RADIO_SPI_CSN_PIN);

    checkRadio(RADIO_SPI_CSN_PIN);

    // Init Pins for SD
    // Config SD?

    // Init Pins for Speaker
    // Config Speaker?
    gpio_set_function(36, GPIO_FUNC_PWM);
    spk_slice = pwm_gpio_to_slice_num(36);
    spk_chan  = pwm_gpio_to_channel(36);
    pwm_hw->slice[spk_slice].csr &= ~PWM_CH0_CSR_DIVMODE_BITS;
    // 125MHz/125=1MHz
    pwm_hw->slice[spk_slice].div = (125 << PWM_CH0_DIV_INT_LSB);
    pwm_hw->slice[spk_slice].top = 1000 - 1;
    pwm_set_chan_level(spk_slice, spk_chan, 0);
    pwm_set_enabled(spk_slice, true);
    gpio_set_function(37, GPIO_FUNC_PWM);
    tick_slice = pwm_gpio_to_slice_num(37);
    pwm_set_clkdiv(tick_slice, 125);
    pwm_set_wrap(tick_slice, 999);
    pwm_clear_irq(tick_slice);
    pwm_set_irq_enabled(tick_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP_0, pwm_tick_irq);
    irq_set_enabled(PWM_IRQ_WRAP_0, true);
    pwm_set_enabled(tick_slice, true);
    uart_puts_string("READY TO READ...");

    // Start Async Measurements in New Process
    // Use Mutex Lock for Safe Radio Access

    // Main Loop
    typedef enum {
        STATE_MENU,
        STATE_LIVE_DATA,
        STATE_HISTORICAL_DATA,
        STATE_SETTINGS,
        STATE_ABOUT
    } State;

    State currentState = STATE_MENU;

    uint8_t historicalDataViewType = 0;
    bool ticks = true;

    uint8_t menuHighlight = 0;
    uint8_t settingsHighlight = 0;
    bool editLock = false;
    menu(menuHighlight);

    int asyncPollInterval = 10000000;
    int livePollInterval = 10000000;

    static uint64_t lastUpdate = 0;
    uint64_t now = time_us_64();

    while (true) {
        uint16_t keyevent = key_pop();

        switch (currentState) {
        case STATE_MENU:
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '2') {
                    menuHighlight = (menuHighlight + 3) % 4;
                    menu(menuHighlight);
                } else if (key == '8') {
                    menuHighlight = (menuHighlight + 1) % 4;
                    menu(menuHighlight);
                } else if (key == '5') {
                    switch (menuHighlight) {
                    case 0:
                        currentState = STATE_LIVE_DATA;
                        // LCD_Clear(LGRAY);
                        liveDataInit();
                        break;
                    case 1:
                        currentState = STATE_HISTORICAL_DATA;
                        // LCD_Clear(GREEN);
                        historicalData(true, 0);
                        break;
                    case 2:
                        currentState = STATE_SETTINGS;
                        // LCD_Clear(MAGENTA);
                        settings(settingsHighlight, false, &asyncPollInterval, &livePollInterval);
                        break;
                    case 3:
                        currentState = STATE_ABOUT;
                        // LCD_Clear(BLUE);
                        about();
                        break;
                    }
                }
            }
            break;
        
        case STATE_LIVE_DATA:
            now = time_us_64();

             if (now - lastUpdate > livePollInterval) {
                liveDataUpdate();
                lastUpdate = now;
            }
            
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(menuHighlight);
                }
            }
            break;

        case STATE_HISTORICAL_DATA:
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(menuHighlight);
                } else if (key == '2') {
                    historicalDataViewType = (historicalDataViewType + 2) % 3;
                    historicalData(ticks, historicalDataViewType);
                } else if (key == '8') {
                    historicalDataViewType = (historicalDataViewType + 1) % 3;
                    historicalData(ticks, historicalDataViewType);
                } else if (key == '5') {
                    ticks = !ticks;
                    historicalData(ticks, historicalDataViewType);
                }
            }
            break;

        case STATE_SETTINGS:
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;

                if (editLock) {
                    if (key == '2') {
                        if (settingsHighlight == 0) {
                            asyncPollInterval += 1000000;
                        } else if (settingsHighlight == 1) {
                            livePollInterval += 1000000;
                        }
                        settings(settingsHighlight, true, &asyncPollInterval, &livePollInterval);
                    } else if (key == '8') {
                        if (settingsHighlight == 0) {
                            if (asyncPollInterval > 1000000) {
                                asyncPollInterval -= 1000000;
                            }
                        } else if (settingsHighlight == 1) {
                            if (livePollInterval > 1000000) {
                                livePollInterval -= 1000000;
                            }
                        }
                        settings(settingsHighlight, true, &asyncPollInterval, &livePollInterval);
                    } else if (key == '5') {
                        editLock = false;
                        settings(settingsHighlight, false, &asyncPollInterval, &livePollInterval);
                    }
                } else {
                    if (key == '1') {
                        currentState = STATE_MENU;
                        menu(menuHighlight);
                    } else if (key == '2') {
                        settingsHighlight = (settingsHighlight + 3) % 2;
                        settings(settingsHighlight, false, &asyncPollInterval, &livePollInterval);
                    } else if (key == '8') {
                        settingsHighlight = (settingsHighlight + 1) % 2;
                        settings(settingsHighlight, false, &asyncPollInterval, &livePollInterval);
                    } else if (key == '5') {
                        settings(settingsHighlight, !editLock, &asyncPollInterval, &livePollInterval);
                        editLock = !editLock;
                    }
                }
            }
            break;

        case STATE_ABOUT:
            if (keyevent & 0x100) {
                char key = keyevent & 0xFF;
                if (key == '1') {
                    currentState = STATE_MENU;
                    menu(menuHighlight);
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