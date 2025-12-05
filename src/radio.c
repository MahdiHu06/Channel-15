// TODO: Confirm SPI instance

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/xosc.h"
#include "hardware/spi.h"
#include "../include/radio.h"
#include "pico/mutex.h"

#define ACK_TIMEOUT_MS  150

static uint8_t tx_seq_num = 0;
static mutex_t radio_mutex;

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

    spi_init(RADIO_SPI_INSTANCE, 125000);
    spi_set_format(RADIO_SPI_INSTANCE, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(csnPin, GPIO_FUNC_SIO);
    gpio_set_dir(csnPin, GPIO_OUT);
    gpio_put(csnPin, 1);

    resetRadio(resetPin);

    mutex_init(&radio_mutex);
}

static void writeRegister_internal(uint CS, uint8_t addr, uint8_t value) {
    gpio_put(CS, 0);
    sleep_us(1);

    uint8_t writeAddr = addr | 0x80;
    spi_write_blocking(RADIO_SPI_INSTANCE, &writeAddr, 1);
    spi_write_blocking(RADIO_SPI_INSTANCE, &value, 1);

    sleep_us(1);
    gpio_put(CS, 1);
}

static uint8_t readRegister_internal(uint CS, uint8_t addr) {
    uint8_t readAddr = addr & 0x7F;
    uint8_t data = 0;

    gpio_put(CS, 0);

    spi_write_blocking(RADIO_SPI_INSTANCE, &readAddr, 1);
    spi_read_blocking(RADIO_SPI_INSTANCE, 0, &data, 1);

    gpio_put(CS, 1);

    return data;
}

void writeRegister(uint CS, uint8_t addr, uint8_t value) {
    mutex_enter_blocking(&radio_mutex);
    writeRegister_internal(CS, addr, value);
    mutex_exit(&radio_mutex);
}

uint8_t readRegister(uint CS, uint8_t addr) {
    mutex_enter_blocking(&radio_mutex);
    uint8_t data = readRegister_internal(CS, addr);
    mutex_exit(&radio_mutex);
    return data;
}

void configRadio(uint CS) {
    mutex_enter_blocking(&radio_mutex);
    // Standby mode
    writeRegister_internal(CS, 0x01, 0x04);
    sleep_ms(10);

    // Clear FIFO
    writeRegister_internal(CS, 0x28, 0x10);

    // Data modulation: Packet mode, FSK, Gaussian filter BT=1.0
    writeRegister_internal(CS, 0x02, 0x00);

    // Bitrate: 4.8 kbps
    writeRegister_internal(CS, 0x03, 0x1A);
    writeRegister_internal(CS, 0x04, 0x0B);

    // Frequency deviation: 25 kHz
    writeRegister_internal(CS, 0x05, 0x01);
    writeRegister_internal(CS, 0x06, 0x9A);

    // Frequency: 915 MHz
    writeRegister_internal(CS, 0x07, 0xE4);
    writeRegister_internal(CS, 0x08, 0xC0);
    writeRegister_internal(CS, 0x09, 0x00);

    // PA config - max power (+20 dBm with PA_BOOST)
    writeRegister_internal(CS, 0x11, 0x9F);

    // PA ramp time
    writeRegister_internal(CS, 0x12, 0x09);

    // OCP - enable with higher limit
    writeRegister_internal(CS, 0x13, 0x2B);

    // LNA: max gain
    writeRegister_internal(CS, 0x18, 0x08);

    // RxBw: 62.5 kHz
    writeRegister_internal(CS, 0x19, 0x42);

    // AFC Bw: 125 kHz
    writeRegister_internal(CS, 0x1A, 0x42);

    // RSSI threshold
    writeRegister_internal(CS, 0x29, 0xE4);

    // Preamble: 8 bytes
    writeRegister_internal(CS, 0x2C, 0x00);
    writeRegister_internal(CS, 0x2D, 0x08);

    // Sync config: on, 2 bytes
    writeRegister_internal(CS, 0x2E, 0x88);
    writeRegister_internal(CS, 0x2F, 0x2D);
    writeRegister_internal(CS, 0x30, 0xD4);

    // Packet config 1: Variable length, CRC ON
    writeRegister_internal(CS, 0x37, 0x90);

    // Payload length max
    writeRegister_internal(CS, 0x38, 0x40);

    // FIFO threshold
    writeRegister_internal(CS, 0x3C, 0x8F);

    // Packet config 2: auto RX restart
    writeRegister_internal(CS, 0x3D, 0x12);

    mutex_exit(&radio_mutex);
}

static void sendPacketRaw_internal(uint CS, uint8_t *data, int length) {
    writeRegister_internal(CS, 0x01, 0x04);
    writeRegister_internal(CS, 0x28, 0x10);
    
    gpio_put(CS, 0);
    
    uint8_t fifoAddr = 0x00 | 0x80;
    spi_write_blocking(RADIO_SPI_INSTANCE, &fifoAddr, 1);
    
    uint8_t len8 = (uint8_t)length;
    spi_write_blocking(RADIO_SPI_INSTANCE, &len8, 1);
    spi_write_blocking(RADIO_SPI_INSTANCE, data, length);
    
    gpio_put(CS, 1);

    writeRegister_internal(CS, 0x01, 0x0C);
    
    printf("TX: Mode after TX start: 0x%02X\n", readRegister_internal(CS, 0x01));

    int timeout = 1000;
    while (!(readRegister_internal(CS, 0x28) & 0x08)) {
        if (--timeout <= 0) {
            printf("TX: PacketSent TIMEOUT!\n");
            break;
        }
        sleep_ms(1);
    }
    
    printf("TX: PacketSent after %d ms, irq2=0x%02X\n", 1000 - timeout, readRegister_internal(CS, 0x28));

    writeRegister_internal(CS, 0x01, 0x04);
    printf("TX: Mode after standby: 0x%02X\n", readRegister_internal(CS, 0x01));
}

void sendPacketRaw(uint CS, uint8_t *data, int length) {
    mutex_enter_blocking(&radio_mutex);
    sendPacketRaw_internal(CS, data, length);
    mutex_exit(&radio_mutex);
}

static void startRadioReceive_internal(uint CS) {
    printf("RX: Switching to RX mode...\n");
    writeRegister_internal(CS, 0x01, 0x04);
    writeRegister_internal(CS, 0x28, 0x10);
    writeRegister_internal(CS, 0x01, 0x10);
    
    int timeout = 100;
    while ((readRegister_internal(CS, 0x27) & 0x80) == 0) {
        if (--timeout <= 0) {
            printf("RX: ModeReady timeout!\n");
            break;
        }
        sleep_us(100);
    }
    
    uint8_t mode = readRegister_internal(CS, 0x01);
    printf("RX: Mode is now 0x%02X (should be 0x10)\n", mode);
}

void startRadioReceive(uint CS) {
    mutex_enter_blocking(&radio_mutex);
    startRadioReceive_internal(CS);
    mutex_exit(&radio_mutex);
}

void sendAck(uint CS, uint8_t seq_num) {
    sleep_ms(15);
    
    uint8_t ack_packet[2];
    ack_packet[0] = PKT_TYPE_ACK;
    ack_packet[1] = seq_num;
    
    printf("TX: Sending ACK for seq %d\n", seq_num);
    sendPacketRaw(CS, ack_packet, 2);
}

bool receivePacketRaw_internal(uint CS, uint8_t *result, uint8_t *length, int timeout_ms) {
    int elapsed = 0;
    
    while (elapsed < timeout_ms) {
        uint8_t irq2 = readRegister_internal(CS, 0x28);
        
        if (irq2 & 0x04) {  // PayloadReady
            gpio_put(CS, 0);
            
            uint8_t fifoAddr = 0x00;
            spi_write_blocking(RADIO_SPI_INSTANCE, &fifoAddr, 1);

            uint8_t packetLength = 0;
            spi_read_blocking(RADIO_SPI_INSTANCE, 0, &packetLength, 1);

            if (packetLength == 0 || packetLength > 64) {
                gpio_put(CS, 1);
                startRadioReceive_internal(CS);
                return false;
            }

            *length = packetLength;
            spi_read_blocking(RADIO_SPI_INSTANCE, 0, result, packetLength);

            gpio_put(CS, 1);
            return true;
        }
        
        sleep_ms(1);
        elapsed++;
    }

    return false;
}

bool receivePacketRaw(uint CS, uint8_t *result, uint8_t *length, int timeout_ms) {
    bool response = false;

    mutex_enter_blocking(&radio_mutex);
    response = receivePacketRaw_internal(CS, result, length, timeout_ms);
    mutex_exit(&radio_mutex);
    return response;
}

bool receivePacketRaw_blocking(uint CS, uint8_t *result, uint8_t *length) {
    mutex_enter_blocking(&radio_mutex);
    while (true) {
        uint8_t irq2 = readRegister_internal(CS, 0x28);

        if (irq2 & 0x04) {  // PayloadReady
            gpio_put(CS, 0);

            uint8_t fifoAddr = 0x00;
            spi_write_blocking(RADIO_SPI_INSTANCE, &fifoAddr, 1);

            uint8_t packetLength = 0;
            spi_read_blocking(RADIO_SPI_INSTANCE, 0, &packetLength, 1);

            if (packetLength == 0 || packetLength > 64) {
                gpio_put(CS, 1);
                startRadioReceive_internal(CS);
                continue;
            }

            *length = packetLength;
            spi_read_blocking(RADIO_SPI_INSTANCE, 0, result, packetLength);

            gpio_put(CS, 1);
            mutex_exit(&radio_mutex);
            return true;
        }

        sleep_ms(1);
    }
}

bool sendDataReliable(uint CS_TX, uint CS_RX, uint8_t *payload, int length) {
    uint8_t packet[66];
    uint8_t response[64];
    uint8_t resp_len;
    
    packet[0] = PKT_TYPE_DATA;
    packet[1] = tx_seq_num;
    memcpy(&packet[2], payload, length);
    
    int total_len = length + 2;
    int max_retries = 3;

    printf("TX: Sending seq %d (len=%d)\n", tx_seq_num, total_len);
    
    for (int retry = 0; retry < max_retries; retry++) {
        mutex_enter_blocking(&radio_mutex);

        sendPacketRaw_internal(CS_TX, packet, total_len);
        
        startRadioReceive_internal(CS_RX);
        
        if (receivePacketRaw_internal(CS_RX, response, &resp_len, 100)) {
            if (resp_len >= 2 && 
                response[0] == PKT_TYPE_ACK && 
                response[1] == tx_seq_num) {
                
                tx_seq_num++;
                printf("TX: ACK for seq %d\n", tx_seq_num - 1);

                mutex_exit(&radio_mutex);
                return true;
            }
        }

        mutex_exit(&radio_mutex);
        
        printf("TX: Retry %d/%d for seq %d\n", retry + 1, max_retries, tx_seq_num);
        sleep_ms(10);
    }
    
    printf("TX: Failed seq %d, giving up\n", tx_seq_num);
    tx_seq_num++;
    return false;
}

void checkRadio(int cs) {
    mutex_enter_blocking(&radio_mutex);
    uint8_t radio_v = readRegister_internal(cs, 0x10);
    mutex_exit(&radio_mutex);

    printf("\n");

    if (radio_v >= 0x21 && radio_v <= 0x25) {
        printf("Radio Connected. Version: 0x%02X\n", radio_v);
    } else {
        printf("Radio Error. Read: 0x%02X\n", radio_v);
    }
}