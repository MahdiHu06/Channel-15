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
        printf("Waiting for packets...\n");
        startRadioReceive(RADIO_SPI_CSN_PIN);

        while (true) {
            // Add a heartbeat to confirm the loop is running
            static int count = 0;
            if (++count % 1000 == 0) {
                printf("Still waiting... (irq2=0x%02X)\n", readRegister(RADIO_SPI_CSN_PIN, 0x28));
            }
            
            uint8_t buffer[64];
            uint8_t length;
            
            receivePacketRaw_blocking(RADIO_SPI_CSN_PIN, buffer, &length);
            
            printf("Received Packet of Length %d\n", length);
            
            // Check if it's a DATA packet (has at least 2 byte header)
            if (length >= 2 && buffer[0] == PKT_TYPE_DATA) {
                uint8_t seq_num = buffer[1];
                uint8_t *payload = &buffer[2];      // Payload starts at byte 2
                uint8_t payload_len = length - 2;   // Subtract header bytes
                
                printf("RX: Got DATA seq %d, payload_len %d\n", seq_num, payload_len);
                
                // Send ACK immediately (IMPORTANT!)
                sendAck(RADIO_SPI_CSN_PIN, seq_num);
                printf("RX: Sent ACK for seq %d\n", seq_num);
                
                // Now process the payload
                if (payload_len >= 1) {
                    uint8_t request_type = payload[0];
                    printf("Request Type: 0x%02X\n", request_type);
                    
                    // Build response
                    uint8_t send_buf[13];
                    int send_len = 1;
                    send_buf[0] = request_type;
                    
                    if ((request_type & 0x07) == 0x07) {
                        // All three requested
                        float temp = read_temp(bme_addr);
                        float pressure = read_pressure(bme_addr);
                        float humidity = read_humidity(bme_addr);
                        memcpy(&send_buf[1], &temp, sizeof(float));
                        memcpy(&send_buf[5], &pressure, sizeof(float));
                        memcpy(&send_buf[9], &humidity, sizeof(float));
                        send_len = 13;
                    } else if (request_type & 0x01) {
                        float temp = read_temp(bme_addr);
                        memcpy(&send_buf[1], &temp, sizeof(float));
                        send_len = 5;
                    } else if (request_type & 0x02) {
                        float pressure = read_pressure(bme_addr);
                        memcpy(&send_buf[1], &pressure, sizeof(float));
                        send_len = 5;
                    } else if (request_type & 0x04) {
                        float humidity = read_humidity(bme_addr);
                        memcpy(&send_buf[1], &humidity, sizeof(float));
                        send_len = 5;
                    }
                    
                    uint8_t response_packet[15];
                    response_packet[0] = PKT_TYPE_DATA;
                    response_packet[1] = seq_num;
                    memcpy(&response_packet[2], send_buf, send_len);
                    
                    sendPacketRaw(RADIO_SPI_CSN_PIN, response_packet, send_len + 2);
                    printf("TX: Sent response\n");
                    
                    // Restart receive mode
                    startRadioReceive(RADIO_SPI_CSN_PIN);
                }
            }
            
            // Restart receive mode
            startRadioReceive(RADIO_SPI_CSN_PIN);
        }
    }
    return 0;
}