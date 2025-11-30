#include "pico/stdlib.h"

void menu(uint8_t highlight);
void liveDataInit(void);
void liveDataUpdate(void);
void historicalData(bool ticks, uint8_t viewType);
void settings(uint8_t highlight, bool editing, int* asyncPollInterval, int* livePollInterval);
void about(void);