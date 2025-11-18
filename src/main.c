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
uint i2c_SDA = 26;
uint i2c_SCL = 27;
i2c_inst_t *i2c = i2c1;


/* Initializing I2C */
void init_i2c() {
    i2c_init(i2c, 115200);
    gpio_set_function(i2c_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_SDA);
    gpio_pull_up(i2c_SCL);
}

int main() {
    init_i2c();
    
    // send to the BMP280 (let's try temp)
    // slave address with RW == 0
    uint8_t slave_addr = BMP280_ADDRESS_ALT * 2;     // * 2 because RW is '0'
    uint8_t regAddr = BMP280_REGISTER_TEMPDATA;
    uint8_t regData = 0;
    uint8_t write_data[3] = {slave_addr, regAddr, regData};
    // reading
    uint8_t data[3] = {};

    for (;;) {    
        i2c_write_blocking(i2c, slave_addr, write_data, 3, true);
        slave_addr++;   // 0x76 to 0x77
        i2c_read_blocking(i2c, slave_addr, data, 3, true);  
        
        for (int i = 0; i < 4; i++) { printf("%d\n", data[i]); }
    }
    return EXIT_SUCCESS;    // should never hit
}