#include "pico/stdlib.h"
#include "hardware/spi.h"

int main() {
    // Init Pins for Display
    // Config Display

    // Init Pins for Keypad
    // Config Keypad

    // Init Pins for Radio
    // Config Radio

    // Init Pins for SD
    // Config SD?

    // Init Pins for Speaker
    // Config Speaker?

    // Start Async Measurements in New Process
    // Use Mutex Lock for Safe Radio Access

    // Main Loop
    // [Display State Drives Many Function Calls]
    // TODO: Figure Out Best way to Share SPI Bus for Two of the Components

    // Restart Loop

    return 0;
}