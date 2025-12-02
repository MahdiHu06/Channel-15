#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/xosc.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/mutex.h"

#define SPI1_SCK_PIN 30
#define SPI1_MISO_RX_PIN 28
#define SPI1_MOSI_TX_PIN 31
#define SPI1_CSN_PIN 29
#define SPI1_RESET_PIN 33

#define SPI0_SCK_PIN 38
#define SPI0_MISO_RX_PIN 36
#define SPI0_MOSI_TX_PIN 39
#define SPI0_CSN_PIN 37
#define SPI0_RESET_PIN 41

// Protocol constants
#define PKT_TYPE_DATA   0x01
#define PKT_TYPE_ACK    0x02
#define ACK_TIMEOUT_MS  150

static mutex_t print_mutex;
static uint8_t tx_seq_num = 0;
static uint8_t last_rx_seq = 255;

void safe_printf(const char* format, ...) {
    mutex_enter_blocking(&print_mutex);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
    mutex_exit(&print_mutex);
}

void resetRadio(uint resetPin) {
    gpio_set_function(resetPin, GPIO_FUNC_SIO);
    gpio_set_dir(resetPin, GPIO_OUT);
    gpio_put(resetPin, 0);
    sleep_us(100);
    
    gpio_set_dir(resetPin, GPIO_IN);
    sleep_ms(5);
}

void initRadio1() {
    gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MISO_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI_TX_PIN, GPIO_FUNC_SPI);

    spi_init(spi1, 125000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(SPI1_CSN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SPI1_CSN_PIN, GPIO_OUT);
    gpio_put(SPI1_CSN_PIN, 1);

    resetRadio(SPI1_RESET_PIN);
}

void initRadio0() {
    gpio_set_function(SPI0_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MISO_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MOSI_TX_PIN, GPIO_FUNC_SPI);

    spi_init(spi0, 125000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    
    gpio_set_function(SPI0_CSN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SPI0_CSN_PIN, GPIO_OUT);
    gpio_put(SPI0_CSN_PIN, 1);

    resetRadio(SPI0_RESET_PIN);
}

void writeRegister(uint CS, uint8_t addr, uint8_t value) {
    gpio_put(CS, 0);

    uint8_t writeAddr = addr | 0x80;
    spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, &writeAddr, 1);
    spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, &value, 1);

    gpio_put(CS, 1);
}

uint8_t readRegister(uint CS, uint8_t addr) {
    uint8_t readAddr = addr & 0x7F;
    uint8_t data = 0;

    gpio_put(CS, 0);

    spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, &readAddr, 1);
    spi_read_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, 0, &data, 1);

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
    writeRegister(CS, 0x02, 0x01);

    // Bitrate: 4.8 kbps
    writeRegister(CS, 0x03, 0x1A);
    writeRegister(CS, 0x04, 0x0B);

    // Frequency deviation: 25 kHz
    writeRegister(CS, 0x05, 0x01);
    writeRegister(CS, 0x06, 0x9A);

    // Frequency: 915 MHz
    writeRegister(CS, 0x07, 0xE4);
    writeRegister(CS, 0x08, 0xC0);
    writeRegister(CS, 0x09, 0x00);

    // PA config - max power
    writeRegister(CS, 0x11, 0x9F);

    // OCP off
    writeRegister(CS, 0x13, 0x1A);

    // LNA: max gain, auto AGC
    writeRegister(CS, 0x18, 0x88);

    // RxBw: 83.3 kHz
    writeRegister(CS, 0x19, 0x41);

    // AFC Bw: wider
    writeRegister(CS, 0x1A, 0x41);

    // AFC auto on, auto clear
    writeRegister(CS, 0x1E, 0x2C);

    // RSSI threshold - more sensitive
    writeRegister(CS, 0x29, 0xFF);

    // Preamble: 32 bytes
    writeRegister(CS, 0x2C, 0x00);
    writeRegister(CS, 0x2D, 0x20);

    // Sync config: on, 2 bytes, allow 1 bit error
    writeRegister(CS, 0x2E, 0x89);
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
    spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, &fifoAddr, 1);
    
    uint8_t len8 = (uint8_t)length;
    spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, &len8, 1);
    spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, data, length);
    
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
            spi_write_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, &fifoAddr, 1);

            uint8_t packetLength = 0;
            spi_read_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, 0, &packetLength, 1);

            if (packetLength == 0 || packetLength > 64) {
                gpio_put(CS, 1);
                startRadioReceive(CS);
                return false;
            }

            *length = packetLength;
            spi_read_blocking((CS == SPI1_CSN_PIN) ? spi1 : spi0, 0, result, packetLength);

            gpio_put(CS, 1);
            return true;
        }
        
        sleep_ms(1);
        elapsed++;
    }
    
    return false;
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
                    safe_printf("TX: ACK for seq %d (%d retries)\n", tx_seq_num - 1, retry);
                } else {
                    safe_printf("TX: ACK for seq %d\n", tx_seq_num - 1);
                }
                return;
            }
        }
        
        retry++;
        if (retry % 10 == 0) {
            safe_printf("TX: Retrying seq %d (%d attempts)\n", tx_seq_num, retry);
            
            // Every 50 retries, do a full radio reset
            if (retry % 50 == 0) {
                safe_printf("TX: Resetting radio\n");
                configRadio(CS_TX);
                configRadio(CS_RX);
            }
        }
        
        sleep_ms(10 + (retry % 10) * 5);
    }
}

void checkRadios() {
    uint8_t radio1_v = readRegister(SPI1_CSN_PIN, 0x10);
    uint8_t radio0_v = readRegister(SPI0_CSN_PIN, 0x10);

    safe_printf("\n");

    if (radio1_v >= 0x21 && radio1_v <= 0x25) {
        safe_printf("Radio 1 Connected. Version: 0x%02X\n", radio1_v);
    } else {
        safe_printf("Radio 1 Error. Read: 0x%02X\n", radio1_v);
    }

    if (radio0_v >= 0x21 && radio0_v <= 0x25) {
        safe_printf("Radio 0 Connected. Version: 0x%02X\n", radio0_v);
    } else {
        safe_printf("Radio 0 Error. Read: 0x%02X\n", radio0_v);
    }
}

void receiverThread() {
    uint8_t buffer[64];
    uint8_t length = 0;

    safe_printf("Receiver Thread Started\n");

    for(;;) {
        startRadioReceive(SPI0_CSN_PIN);
        
        if (receivePacketRaw(SPI0_CSN_PIN, buffer, &length, 50)) {
            
            if (length >= 2 && buffer[0] == PKT_TYPE_DATA) {
                uint8_t seq_num = buffer[1];
                uint8_t *payload = &buffer[2];
                uint8_t payload_len = length - 2;
                
                // Send ACK immediately
                sendAck(SPI0_CSN_PIN, seq_num);
                
                // Check if this is a new packet (not the same as last one)
                if (seq_num != last_rx_seq) {
                    last_rx_seq = seq_num;
                    payload[payload_len] = '\0';
                    safe_printf("RX: [seq=%d] %s\n", seq_num, payload);
                }
                
            }
        }
    }
}

int main() {
    stdio_init_all();
    mutex_init(&print_mutex);

    initRadio1();
    initRadio0();

    configRadio(SPI1_CSN_PIN);
    configRadio(SPI0_CSN_PIN);

    checkRadios();

    multicore_launch_core1(receiverThread);
    
    sleep_ms(500);

    int counter = 0;

    while (true) {
        char message[32];
        int msgLength = snprintf(message, sizeof(message), "Hello %d from Radio 1", counter++);

        sendDataReliable(SPI1_CSN_PIN, SPI1_CSN_PIN, (uint8_t*)message, msgLength);
        
        safe_printf("TX: Sent: %s\n", message);

        sleep_ms(500);
    }

    return 0;
}

