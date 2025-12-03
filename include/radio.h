// TODO: Confirm pin numbers

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// Pin definitions
#define RADIO_SPI_SCK_PIN 38
#define RADIO_SPI_MISO_RX_PIN 36
#define RADIO_SPI_MOSI_TX_PIN 39
#define RADIO_SPI_CSN_PIN 37
#define RADIO_SPI_RESET_PIN 41
#define RADIO_SPI_INSTANCE spi0

#define PKT_TYPE_DATA   0x01
#define PKT_TYPE_ACK    0x02

// Function declarations
void resetRadio(uint resetPin);
void initRadio(int sckPin, int misoPin, int mosiPin, int csnPin, int resetPin);
void writeRegister(uint CS, uint8_t addr, uint8_t value);
uint8_t readRegister(uint CS, uint8_t addr);
void configRadio(uint CS);
void startRadioReceive(uint CS);

void sendPacketRaw(uint CS, uint8_t *data, int length);
bool receivePacketRaw(uint CS, uint8_t *result, uint8_t *length, int timeout_ms);
bool receivePacketRaw_blocking(uint CS, uint8_t *result, uint8_t *length);

void sendAck(uint CS, uint8_t seq_num);
bool sendDataReliable(uint CS_TX, uint CS_RX, uint8_t *payload, int length);

void checkRadio(int cs);