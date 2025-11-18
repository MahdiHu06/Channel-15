#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// MCU includes
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"

////////////////////////////////////////////////////////////////////////

/* Definitions */
#define BMP280_ADDRESS (0x77) /**< The default I2C address for the sensor. */
#define BMP280_ADDRESS_ALT                                                     \
  (0x76)                     /**< Alternative I2C address for the sensor. */
#define BMP280_CHIPID (0x58) /**< Default chip ID. */

/* Available Registers */
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1 /**< R calibration = 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA

// Functions
void init_i2c();

// Pin numbers for I2C
uint i2c_SDA = 30;
uint i2c_SCL = 31;
i2c_inst_t *i2c = i2c1;

#define BUFSIZE 32
char serbuf[BUFSIZE];
int seridx = 0;
int newline_seen = 0;

// add this here so that compiler does not complain about implicit function
void uart_rx_handler();
void init_uart() { 
    uart_init(uart0, 115200); 
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
}
void init_uart_irq() {
    // disable FIFO
    uart_set_fifo_enabled(uart0, false);

    // mask the interrupt
    uart0_hw->imsc = (1 << 4); 
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_handler);
    
    irq_set_enabled(UART0_IRQ, true);
}

void uart_rx_handler() {
    // Acknowledge Interrupt
    uart0_hw->icr = 1 << 4;

    if (seridx == BUFSIZE) { return; }
    
    char c = uart0_hw->dr;      // get character

    // check for what the character is
    if (c == 0x0A) { newline_seen = 1; }
    if ((c == 0x8) && (seridx > 0)) {
        putchar(c);     // backspace
        putchar(0x20);  // space
        putchar(c);     // backspace
        seridx--;   
        serbuf[seridx] = '\0';
        return;
    }

    // Otherwise
    serbuf[seridx] = c;
    uart_write_blocking(uart0, &serbuf[seridx], 1);
    seridx++;
}

int _read(__unused int handle, char *buffer, int length) {
    while (newline_seen == 0) { sleep_ms(5); }
    newline_seen = 0;
    for (int i = 0; i < seridx; i++) {
        if (i == length) { break; }
        buffer[i] = serbuf[i];
    }
    uint idx = seridx;
    seridx = 0;
    return idx;
}

int _write(__unused int handle, char *buffer, int length) {
    for (int i = 0; i < length; i++) { uart_write_blocking(uart0, &buffer[i], 1); }
    return length;
}

/* Initializing I2C */
void init_i2c() {
    i2c_init(i2c, 100 * 1000);
    gpio_set_function(i2c_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_SDA);
    gpio_pull_up(i2c_SCL);
}
void bmp_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

void bmp_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, size_t length) {
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, buffer, length, false);
}
int main() {
    init_uart();
    init_uart_irq();

    setbuf(stdout, NULL); // disable buffering for stdout
    // printf("\nHello World!\n");
    init_i2c();


    uint8_t bmp_addr = BMP280_ADDRESS_ALT;
    sleep_ms(500);

    uint8_t chip_id = 0x0;
    bmp_read_bytes(bmp_addr, BMP280_REGISTER_CHIPID, &chip_id, 1);
    printf("Chip ID: 0x%X (expected 0x%X)\n", chip_id, BMP280_CHIPID);
    

    bmp_write_reg(bmp_addr, BMP280_REGISTER_CONTROL, 0x27);
    for (;;) {        
        // replace new line with null
        uint8_t data[3];
        bmp_read_bytes(bmp_addr, BMP280_REGISTER_TEMPDATA, data, 3);
        int32_t temp_data = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
        printf("Temp: 0x%X\n", temp_data);
        
        sleep_ms(1000);
   }
   return EXIT_SUCCESS;    // should never hit
}
