#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "../include/radio.h"
#include "../include/bme280.h"

int main() {
    stdio_init_all();

    // Init Pins for Radio
    initRadio(RADIO_SPI_SCK_PIN, RADIO_SPI_MISO_RX_PIN, RADIO_SPI_MOSI_TX_PIN, RADIO_SPI_CSN_PIN, RADIO_SPI_RESET_PIN);
    // Config Radio
    configRadio(RADIO_SPI_CSN_PIN);

    checkRadio(RADIO_SPI_CSN_PIN);

    // Init Pins for Sensor
    init_i2c();

    // Config Sensor
    uint8_t bme_addr = BME280_ADDRESS_ALT;
    sleep_ms(500);

    bme_write_reg(bme_addr, BME280_REGISTER_CONTROL, 0x27);

    uint8_t chip_id = 0x0;
    bme_read_bytes(bme_addr, BME280_REGISTER_CHIPID, &chip_id, 1);
    printf("Chip ID: 0x%02X (expected 0x60)\n", chip_id);

    bme_read_calibration(bme_addr);

    // Begin Main Loop
    while (true) {
        // Start Radio Receive
        startRadioReceive(RADIO_SPI_CSN_PIN);

        // Wait for Request from Radio
        uint8_t response[256];
        uint8_t resp_len;
        float measurement;
        
        receivePacketRaw_blocking(RADIO_SPI_CSN_PIN, response, &resp_len);
        printf("Received Packet of Length %d\n", resp_len);
        printf("Request Type: 0x%02X\n", response[0]);

        // Request Router Based on Measurement Type and Unit
        /*
        Bit 0: Temperature
        Bit 1: Pressure
        Bit 2: Humidity
        */
        if (resp_len < 1) {
            printf("Invalid Packet Length\n");
            continue;
        } else {
            uint8_t send_buf[13];
            send_buf[0] = response[0];

            int send_len = 1;

            if ((response[0] & 0x07) == 0x07) {
                float temp = read_temp(bme_addr);
                float pressure = read_pressure(bme_addr);
                float humidity = read_humidity(bme_addr);
                memcpy(&send_buf[1],  &temp,     sizeof(float));
                memcpy(&send_buf[5],  &pressure, sizeof(float));
                memcpy(&send_buf[9],  &humidity, sizeof(float));
                send_len = 13;
            } else if (response[0] & 0x01) {
                // Temperature Requested
                float measurement = read_temp(bme_addr);
                memcpy(&send_buf[1], &measurement, sizeof(float));
                send_len = 5;
            } else if (response[0] & 0x02) {
                // Pressure Requested
                float measurement = read_pressure(bme_addr);
                memcpy(&send_buf[1], &measurement, sizeof(float));
                send_len = 5;
            } else if (response[0] & 0x04) {
                // Humidity Requested
                float measurement = read_humidity(bme_addr);
                memcpy(&send_buf[1], &measurement, sizeof(float));
                send_len = 5;
            } else {
                printf("Invalid Measurement Request\n");
                continue;
            }

            sendDataReliable(RADIO_SPI_CSN_PIN, RADIO_SPI_CSN_PIN, send_buf, 5);
        }
    }
    return 0;
}