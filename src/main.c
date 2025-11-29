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
#define BME280_ADDRESS (0x77) /**< The default I2C address for the sensor. */
#define BME280_ADDRESS_ALT                                                     \
  (0x76)                     /**< Alternative I2C address for the sensor. */

/* Available Registers */
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C

#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E
#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7

#define BME280_REGISTER_CHIPID 0xD0
#define BME280_REGISTER_VERSION 0xD1
#define BME280_REGISTER_SOFTRESET 0xE0

#define BME280_REGISTER_CAL26 0xE1 // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID 0xF2
#define BME280_REGISTER_STATUS 0XF3
#define BME280_REGISTER_CONTROL 0xF4
#define BME280_REGISTER_CONFIG 0xF5
#define BME280_REGISTER_PRESSUREDATA 0xF7
#define BME280_REGISTER_TEMPDATA 0xFA
#define BME280_REGISTER_HUMIDDATA 0xFD

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
void bme_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

void bme_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, size_t length) {
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, buffer, length, false);
}


// Conversion stuff
int32_t t_fine;

// temp
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;

// pressure
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// humidity
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4, dig_H5;
int8_t   dig_H6;

void bme_read_calibration(uint8_t addr) {
    uint8_t buf[32];

    // Temperature calibration
    bme_read_bytes(addr, BME280_REGISTER_DIG_T1, buf, 6);
    dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
    dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);

    bme_read_bytes(addr, BME280_REGISTER_DIG_P1, buf, 18);
    
    // NOTE: Everything here is little-endian
    dig_P1 = (uint16_t)((buf[1] << 8) | buf[0]);
    dig_P2 = (int16_t)((buf[3] << 8) | buf[2]);
    dig_P3 = (int16_t)((buf[5] << 8) | buf[4]);
    dig_P4 = (int16_t)((buf[7] << 8) | buf[6]);
    dig_P5 = (int16_t)((buf[9] << 8) | buf[8]);
    dig_P6 = (int16_t)((buf[11] << 8) | buf[10]);
    dig_P7 = (int16_t)((buf[13] << 8) | buf[12]);
    dig_P8 = (int16_t)((buf[15] << 8) | buf[14]);
    dig_P9 = (int16_t)((buf[17] << 8) | buf[16]);

    // Humidity
    bme_read_bytes(addr, BME280_REGISTER_DIG_H1, &dig_H1, 1);
    uint8_t hbuf[7];
    bme_read_bytes(addr, BME280_REGISTER_DIG_H2, hbuf, 7);

    dig_H2 = (int16_t)((hbuf[1] << 8) | hbuf[0]);                       // E1, E2
    dig_H3 = hbuf[2];                                                   // E3
    dig_H4 = (int16_t)((hbuf[3] << 4) | (hbuf[4] & 0x0F));              // E4 MSB + E5[3:0]
    dig_H5 = (int16_t)((hbuf[5] << 4) | (hbuf[4] >> 4));                // E6 MSB + E5[7:4]
    dig_H6 = (int8_t)hbuf[6];                                           // E7
}

/* Start Calculating Values */
float calculate_temp(int32_t adc_T, int F) {
    float var1, var2, T;

    var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
    var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f) *
           (((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f)) *
           ((float)dig_T3);

    t_fine = (int32_t)(var1 + var2);

    T = (var1 + var2) / 5120.0f;

    // check if fahrenheit or celsius
    T = F ? ((T * ( 9.0 / 5.0 )) + 32) : T;
    return T;
}

float calculate_pressure(int32_t adc_P) {
    // Needs t_fine from the last temperature reading
    double var1, var2, p;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * (double)dig_P6 / 32768.0;
    var2 = var2 + var1 * (double)dig_P5 * 2.0;
    var2 = (var2 / 4.0) + ((double)dig_P4 * 65536.0);

    var1 = ((double)dig_P3 * var1 * var1 / 524288.0 +
            (double)dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (double)dig_P1;

    if (var1 == 0.0) {
        return 0; // avoid div-by-zero
    }

    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;

    var1 = (double)dig_P9 * p * p / 2147483648.0;
    var2 = p * (double)dig_P8 / 32768.0;
    p = p + (var1 + var2 + (double)dig_P7) / 16.0;

    return (float)p;   // Pascals
}

float calculate_humidity(int32_t adc_H) {
    double var_H;

    var_H = ((double)t_fine) - 76800.0;

    var_H = ((double)adc_H - ( (double)dig_H4 * 64.0 + (double)dig_H5 / 16384.0 * var_H )) *
            ((double)dig_H2 / 65536.0 * ( 1.0 + (double)dig_H6 / 67108864.0 * var_H *(1.0 + (double)dig_H3 / 67108864.0 * var_H))
    );

    var_H = var_H * (1.0 - (double)dig_H1 * var_H / 524288.0);

    if (var_H > 100.0) var_H = 100.0;
    if (var_H < 0.0)   var_H = 0.0;

    return (float)var_H;   // % relative humidity
}

/* Functions for reading */
void read_temp(int8_t bme_addr) {
    uint8_t raw_temp_data[3];
    bme_read_bytes(bme_addr, BME280_REGISTER_TEMPDATA, raw_temp_data, 3);
    int32_t temp_data = ((int32_t)raw_temp_data[0] << 12) | ((int32_t)raw_temp_data[1] << 4) | ((int32_t)raw_temp_data[2] >> 4);
    
    // convert raw data
    int isF = 0;
    float temp = calculate_temp(temp_data, isF);
    char rep = isF ? 'F' : 'C';
    printf("Temp: %.2f °%c\n", temp, rep);
}
void read_pressure(int8_t bme_addr) {
    uint8_t raw_pressure_data[3];
    bme_read_bytes(bme_addr, BME280_REGISTER_PRESSUREDATA, raw_pressure_data, 3);
    int32_t pressure_data = ((int32_t)raw_pressure_data[0] << 12) | ((int32_t)raw_pressure_data[1] << 4) | ((int32_t)raw_pressure_data[2] >> 4);
    float pressure = calculate_pressure(pressure_data);
    pressure /= 1000;
    printf("Pressure: %.2f kP\n", pressure);
}

void read_humidity(int8_t bme_addr) {
    uint8_t raw_humidity_data[2];
    bme_read_bytes(bme_addr, BME280_REGISTER_HUMIDDATA, raw_humidity_data, 2);

    int32_t adc_H = ((int32_t)raw_humidity_data[0] << 8) | ((int32_t)raw_humidity_data[1]);
    float humidity = calculate_humidity(adc_H);   // %RH

    printf("Humidity: %.2f%%\n", humidity);
}

int main() {
    init_uart();
    init_uart_irq();

    setbuf(stdout, NULL); // disable buffering for stdout
    // printf("\nHello World!\n");
    init_i2c();


    uint8_t bme_addr = BME280_ADDRESS_ALT;
    sleep_ms(500);

    

    bme_write_reg(bme_addr, BME280_REGISTER_CONTROL, 0x27);


    uint8_t chip_id = 0x0;
    bme_read_bytes(bme_addr, BME280_REGISTER_CHIPID, &chip_id, 1);
    printf("Chip ID: 0x%02X (expected 0x60)\n", chip_id);
    printf("-----------------------------------------------\n");
    // Read calibration coefficients
    bme_read_calibration(bme_addr);

    for (;;) {        
        // replace new line with null
        read_temp(bme_addr);
        read_pressure(bme_addr);
        read_humidity(bme_addr);
        printf("-----------------------------------------------\n");
        sleep_ms(1000);
   }
   return EXIT_SUCCESS;    // should never hit
}