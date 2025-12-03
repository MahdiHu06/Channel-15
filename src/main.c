#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ff.h"
#include "diskio.h"
#include "sdcard.h"
#include <stdio.h>
#include <string.h>

/*******************************************************************/

#define SD_MISO 8
#define SD_CS   9
#define SD_SCK  10
#define SD_MOSI 11



/*******************************************************************/


#define SPI_INST spi1

void init_spi_sdcard() {
    // Initialize SPI at 400 KHz for SD card initialization
    spi_init(SPI_INST, 400 * 1000);
    
    // Configure SCK, MOSI, MISO as SPI pins
    gpio_set_function(SD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SD_MISO, GPIO_FUNC_SPI);
    
    // Configure CS as a regular GPIO pin for manual control
    gpio_init(SD_CS);
    gpio_set_dir(SD_CS, GPIO_OUT);
    gpio_put(SD_CS, 1);  // Set CS high (inactive)
    
    // Set SPI format: 8-bit data, CPOL=0, CPHA=0
    spi_set_format(SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

void disable_sdcard() {
    // Set CS high (inactive)
    gpio_put(SD_CS, 1);
    
    // Send 0xFF to give SD card some clock cycles to finish up
    uint8_t dummy = 0xFF;
    spi_write_blocking(SPI_INST, &dummy, 1);
    
    // "Release" the MOSI line by making it a GPIO and forcing it high
    // This is required by some SD card specifications
    gpio_init(SD_MOSI);
    gpio_set_dir(SD_MOSI, GPIO_OUT);
    gpio_put(SD_MOSI, 1);
}

void enable_sdcard() {
    // "Take control" of MOSI line by making it an SPI pin again
    gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);
    
    // Set CS low (active)
    gpio_put(SD_CS, 0);
}

void sdcard_io_high_speed() {
    // Change baudrate to 12 MHz for faster transfers
    // You can experiment with higher speeds (up to 24 MHz)
    spi_set_baudrate(SPI_INST, 12 * 1000 * 1000);
}

void init_sdcard_io() {
    // Initialize SPI for SD card
    init_spi_sdcard();
    
    // Put SD card into inactive state
    disable_sdcard();
}

/*******************************************************************/

void init_uart();
void init_uart_irq();
void date(int argc, char *argv[]);
void command_shell();

int main() {
    // Initialize the standard input/output library
    // Initialize UART first
    init_uart();
    init_uart_irq();
    
    // Small delay for UART to stabilize
    sleep_ms(200);
    
    printf("\n\n=== SD Card System Starting ===\n");

    
    // Initialize SD card SPI
    printf("Initializing SD Card SPI...\n");
    init_sdcard_io();

    // Dummy 10x3 logged data
    int dummyData[10][3] = {
        {10, 20, 30},
        {11, 21, 31},
        {12, 22, 32},
        {13, 23, 33},
        {14, 24, 34},
        {15, 25, 35},
        {16, 26, 36},
        {17, 27, 37},
        {18, 28, 38},
        {19, 29, 39}
    };

    printf("Calling log_data_csv...\n");
    FRESULT fr = log_data_csv("log.csv", dummyData);

    if (fr == FR_OK) {
        printf("Logging OK!\n");
    } else {
        printf("Logging failed: %d\n", fr);
    }

    /*
    printf("Starting command shell...\n");
    // Start command shell (this will call display_init())
    command_shell();
    */

    for(;;);
}