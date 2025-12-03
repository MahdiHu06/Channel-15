// TODO: Confirm pin numbers

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#define RADIO_SPI_SCK_PIN 30
#define RADIO_SPI_MISO_RX_PIN 28
#define RADIO_SPI_MOSI_TX_PIN 31
#define RADIO_SPI_CSN_PIN 29
#define RADIO_SPI_RESET_PIN 33

#define PKT_TYPE_DATA   0x01
#define PKT_TYPE_ACK    0x02

void resetRadio(uint resetPin);
void initRadio(int sckPin, int misoPin, int mosiPin, int csnPin, int resetPin);
void writeRegister(uint CS, uint8_t addr, uint8_t value);
uint8_t readRegister(uint CS, uint8_t addr);
void configRadio(uint CS);
void startRadioReceive(uint CS);

void sendPacketRaw(uint CS, uint8_t *data, int length);
bool receivePacketRaw_blocking(uint CS, uint8_t *result, uint8_t *length);
bool receivePacketRaw(uint CS, uint8_t *result, uint8_t *length, int timeout_ms);
void sendAck(uint CS, uint8_t seq_num);
void sendDataReliable(uint CS_TX, uint CS_RX, uint8_t *payload, int length);

void checkRadio(int cs);