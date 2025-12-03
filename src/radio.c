// TODO: Confirm SPI instance

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/xosc.h"
#include "hardware/spi.h"
#include "../include/radio.h"

// Protocol constants
#define ACK_TIMEOUT_MS  150

static uint8_t tx_seq_num = 0;
static uint8_t last_rx_seq = 255;

void resetRadio(uint resetPin) {
    gpio_set_function(resetPin, GPIO_FUNC_SIO);
    gpio_set_dir(resetPin, GPIO_OUT);
    gpio_put(resetPin, 0);
    sleep_us(100);
    
    gpio_set_dir(resetPin, GPIO_IN);
    sleep_ms(5);
}

void initRadio(int sckPin, int misoPin, int mosiPin, int csnPin, int resetPin) {
    gpio_set_function(sckPin, GPIO_FUNC_SPI);
    gpio_set_function(misoPin, GPIO_FUNC_SPI);
    gpio_set_function(mosiPin, GPIO_FUNC_SPI);

    spi_init(spi1, 125000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(csnPin, GPIO_FUNC_SIO);
    gpio_set_dir(csnPin, GPIO_OUT);
    gpio_put(csnPin, 1);

    resetRadio(resetPin);
}

void writeRegister(uint CS, uint8_t addr, uint8_t value) {
    gpio_put(CS, 0);

    uint8_t writeAddr = addr | 0x80;
    spi_write_blocking(spi1, &writeAddr, 1);
    spi_write_blocking(spi1, &value, 1);

    gpio_put(CS, 1);
}

uint8_t readRegister(uint CS, uint8_t addr) {
    uint8_t readAddr = addr & 0x7F;
    uint8_t data = 0;

    gpio_put(CS, 0);

    spi_write_blocking(spi1, &readAddr, 1);
    spi_read_blocking(spi1, 0, &data, 1);

    gpio_put(CS, 1);

    return data;
}

void configRadio(uint CS) {
    // Standby mode
    writeRegister(CS, 0x01, 0x04);
    sleep_ms(10);

    // Clear FIFO
    writeRegister(CS, 0x28, 0x10);

    // Data modulation: Packet mode, FSK, Gaussian filter BT=1.0
    writeRegister(CS, 0x02, 0x00);

    // Bitrate: 4.8 kbps (faster than 1.2 kbps)
    writeRegister(CS, 0x03, 0x1A);
    writeRegister(CS, 0x04, 0x0B);

    // Frequency deviation: 25 kHz
    writeRegister(CS, 0x05, 0x01);
    writeRegister(CS, 0x06, 0x9A);

    // Frequency: 915 MHz
    writeRegister(CS, 0x07, 0xE4);
    writeRegister(CS, 0x08, 0xC0);
    writeRegister(CS, 0x09, 0x00);

    // PA config - max power (+20 dBm with PA_BOOST)
    writeRegister(CS, 0x11, 0x9F);

    // PA ramp time
    writeRegister(CS, 0x12, 0x09);

    // OCP - enable with higher limit
    writeRegister(CS, 0x13, 0x2B);

    // LNA: max gain
    writeRegister(CS, 0x18, 0x08);

    // RxBw: 62.5 kHz (wider for faster bitrate)
    writeRegister(CS, 0x19, 0x42);

    // AFC Bw: 125 kHz
    writeRegister(CS, 0x1A, 0x42);

    // RSSI threshold
    writeRegister(CS, 0x29, 0xE4);

    // Preamble: 8 bytes (shorter for faster sync)
    writeRegister(CS, 0x2C, 0x00);
    writeRegister(CS, 0x2D, 0x08);

    // Sync config: on, 2 bytes
    writeRegister(CS, 0x2E, 0x88);
    writeRegister(CS, 0x2F, 0x2D);
    writeRegister(CS, 0x30, 0xD4);

    // Packet config 1: Variable length, CRC ON
    writeRegister(CS, 0x37, 0x90);

    // Payload length max
    writeRegister(CS, 0x38, 0x40);

    // FIFO threshold
    writeRegister(CS, 0x3C, 0x8F);

    // Packet config 2: auto RX restart
    writeRegister(CS, 0x3D, 0x12);
}

void startRadioReceive(uint CS) {
    writeRegister(CS, 0x01, 0x04);
    writeRegister(CS, 0x28, 0x10);
    writeRegister(CS, 0x01, 0x10);
    sleep_ms(1);
}

void sendPacketRaw(uint CS, uint8_t *data, int length) {
    // Go to standby and clear FIFO
    writeRegister(CS, 0x01, 0x04);
    writeRegister(CS, 0x28, 0x10);
    
    gpio_put(CS, 0);
    
    uint8_t fifoAddr = 0x00 | 0x80;
    spi_write_blocking(spi1, &fifoAddr, 1);
    
    uint8_t len8 = (uint8_t)length;
    spi_write_blocking(spi1, &len8, 1);
    spi_write_blocking(spi1, data, length);
    
    gpio_put(CS, 1);

    writeRegister(CS, 0x01, 0x0C);

    // Wait for PacketSent with timeout
    int timeout = 1000;
    while (!(readRegister(CS, 0x28) & 0x08)) {
        if (--timeout <= 0) break;
        sleep_ms(1);
    }

    writeRegister(CS, 0x01, 0x04);
}

bool receivePacketRaw(uint CS, uint8_t *result, uint8_t *length, int timeout_ms) {
    int elapsed = 0;
    
    while (elapsed < timeout_ms) {
        uint8_t irq2 = readRegister(CS, 0x28);
        
        if (irq2 & 0x04) {  // PayloadReady
            gpio_put(CS, 0);

            uint8_t fifoAddr = 0x00;
            spi_write_blocking(spi1, &fifoAddr, 1);

            uint8_t packetLength = 0;
            spi_read_blocking(spi1, 0, &packetLength, 1);

            if (packetLength == 0 || packetLength > 64) {
                gpio_put(CS, 1);
                startRadioReceive(CS);
                return false;
            }

            *length = packetLength;
            spi_read_blocking(spi1, 0, result, packetLength);

            gpio_put(CS, 1);
            return true;
        }
        
        sleep_ms(1);
        elapsed++;
    }
    
    return false;
}

bool receivePacketRaw_blocking(uint CS, uint8_t *result, uint8_t *length) {
    while (true) {
        uint8_t irq2 = readRegister(CS, 0x28);

        if (irq2 & 0x04) {  // PayloadReady
            gpio_put(CS, 0);

            uint8_t fifoAddr = 0x00;
            spi_write_blocking(spi1, &fifoAddr, 1);

            uint8_t packetLength = 0;
            spi_read_blocking(spi1, 0, &packetLength, 1);

            if (packetLength == 0 || packetLength > 64) {
                gpio_put(CS, 1);
                startRadioReceive(CS);
                continue;
            }

            *length = packetLength;
            printf("RX: Packet Length: %d\n", packetLength);
            spi_read_blocking(spi1, 0, result, packetLength);

            if (packetLength >= 2 && result[0] == PKT_TYPE_DATA) {
                last_rx_seq = result[1];
            }

            gpio_put(CS, 1);
            return true;
        }

        sleep_ms(1);
    }
}

void sendAck(uint CS, uint8_t seq_num) {
    uint8_t ack_packet[2];
    ack_packet[0] = PKT_TYPE_ACK;
    ack_packet[1] = seq_num;
    
    sendPacketRaw(CS, ack_packet, 2);
}

void sendDataReliable(uint CS_TX, uint CS_RX, uint8_t *payload, int length) {
    uint8_t packet[66];
    uint8_t response[64];
    uint8_t resp_len;
    
    // Build packet
    packet[0] = PKT_TYPE_DATA;
    packet[1] = tx_seq_num;
    memcpy(&packet[2], payload, length);
    
    int total_len = length + 2;
    int retry = 0;
    
    while (true) {
        // Send the packet
        sendPacketRaw(CS_TX, packet, total_len);
        
        startRadioReceive(CS_RX);
        
        // Wait for ACK with shorter timeout
        if (receivePacketRaw(CS_RX, response, &resp_len, 100)) {
            // Check if it's an ACK for our sequence number
            if (resp_len >= 2 && 
                response[0] == PKT_TYPE_ACK && 
                response[1] == tx_seq_num) {
                
                tx_seq_num++;
                if (retry > 0) {
                    printf("TX: ACK for seq %d (%d retries)\n", tx_seq_num - 1, retry);
                } else {
                    printf("TX: ACK for seq %d\n", tx_seq_num - 1);
                }
                return;
            }
        }
        
        retry++;
        if (retry % 10 == 0) {
            printf("TX: Retrying seq %d (%d attempts)\n", tx_seq_num, retry);
            
            // Every 50 retries, do a full radio reset
            if (retry % 50 == 0) {
                printf("TX: Resetting radio\n");
                configRadio(CS_TX);
                configRadio(CS_RX);
            }
        }
        
        sleep_ms(10 + (retry % 10) * 5);
    }
}

void sendResponse(uint CS, uint8_t *payload, int length) {
    uint8_t packet[66];
    
    // Build packet (use same seq_num as request)
    packet[0] = PKT_TYPE_DATA;
    packet[1] = last_rx_seq;  // Echo the received sequence number
    memcpy(&packet[2], payload, length);
    
    sendPacketRaw(CS, packet, length + 2);
    
    // Go back to receive mode
    startRadioReceive(CS);
}

void checkRadio(int cs) {
    uint8_t radio_v = readRegister(cs, 0x10);

    printf("\n");

    if (radio_v >= 0x21 && radio_v <= 0x25) {
        printf("Radio Connected. Version: 0x%02X\n", radio_v);
    } else {
        printf("Radio Error. Read: 0x%02X\n", radio_v);
    }
}