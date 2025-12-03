#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "../include/views.h"
#include <stdio.h>
#include "../include/helper.h"
#include "../include/radio.h"
#include "../include/lcd.h"

void menu(uint8_t highlight) {
    LCD_Clear(0x0000);

    const char* options[] = {"Live Data", "Historical Data", "Settings", "About"};
    for (uint8_t i = 0; i < 4; i++) {
        if (i == highlight) {
            LCD_DrawFillRectangle(0, i * 80, 270, 80 + i * 80, 0x01CF);
            LCD_DrawString(10, 32 + i * 80, 0xFFFF, 0x01CF, options[i], 16, 1);
        } else {
            LCD_DrawString(10, 32 + i * 80, 0xFFFF, 0x0000, options[i], 16, 1);
        }
    }
}

void liveDataInit(void) {
    LCD_Clear(0x0000);

    LCD_DrawString(10, 0, 0xFFFF, 0x0000, "Pressure", 16, 1);
    char pressureStr[16];
    sprintf(pressureStr, "%d hPa", randomFloat(950, 1050));
    LCD_DrawString(10, 40, 0xFFFF, 0x0000, pressureStr, 16, 1);

    LCD_DrawString(10, 100, 0xFFFF, 0x0000, "Temperature", 16, 1);
    char tempStr[16];
    sprintf(tempStr, "%d F", randomFloat(-115, 115));
    LCD_DrawString(10, 140, 0xFFFF, 0x0000, tempStr, 16, 1);

    LCD_DrawString(10, 200, 0xFFFF, 0x0000, "Humidity", 16, 1);
    char humidityStr[16];
    sprintf(humidityStr, "%d%%", randomFloat(0, 100));
    LCD_DrawString(10, 240, 0xFFFF, 0x0000, humidityStr, 16, 1);
}

void liveDataUpdate(void) {
    /*
    Bit 0: Temperature
    Bit 1: Pressure
    Bit 2: Humidity
    */

    uint8_t request_buf[1];
    request_buf[0] = 0x07; // Request all data

    sendDataReliable(RADIO_SPI_CSN_PIN, RADIO_SPI_CSN_PIN, request_buf, 1);

    uint8_t response_buf[13];
    uint8_t response_len;
    
    
    // Use non-blocking with timeout (e.g., 500ms)
    if (!receivePacketRaw(RADIO_SPI_CSN_PIN, response_buf, &response_len, 500)) {
        // No response received - display error or keep old values
        printf("No response from sensor\n");
        return;
    }

    // Check if it's a DATA packet and strip header
    if (response_len < 2 || response_buf[0] != PKT_TYPE_DATA) {
        printf("Invalid response packet\n");
        return;
    }

    uint8_t *payload = &response_buf[2];
    uint8_t payload_len = response_len - 2;

    float pressure = 0;
    float temp = 0;
    float humidity = 0;
    if (response_len == 13 && response_buf[0] == 0x07) {
        memcpy(&temp,     &response_buf[1],  sizeof(float));
        memcpy(&pressure, &response_buf[5],  sizeof(float));
        memcpy(&humidity, &response_buf[9],  sizeof(float));
    } else {
        printf("Unexpected payload length: %d\n", payload_len);
        return;
    }

    LCD_DrawFillRectangle(10, 40, 240, 70, 0x0000);
    char pressureStr[16];
    sprintf(pressureStr, "%d hPa", pressure);
    LCD_DrawString(10, 40, 0xFFFF, 0x0000, pressureStr, 16, 1);

    LCD_DrawFillRectangle(10, 140, 240, 170, 0x0000);
    char tempStr[16];
    sprintf(tempStr, "%d F", temp);
    LCD_DrawString(10, 140, 0xFFFF, 0x0000, tempStr, 16, 1);

    LCD_DrawFillRectangle(10, 240, 240, 270, 0x0000);
    char humidityStr[16];
    sprintf(humidityStr, "%d%%", humidity);
    LCD_DrawString(10, 240, 0xFFFF, 0x0000, humidityStr, 16, 1);
}

void historicalData(bool ticks, uint8_t viewType) {
    int32_t loggedData[10][3];
    int32_t predictedData[10][3];

    generateRandomArray(loggedData, -100, 100);
    generateRandomArray(predictedData, -100, 100);

    LCD_Clear(0x0000);

    // Axis
    LCD_DrawFillRectangle(43, 6, 44, 276, 0xFFFF);
    LCD_DrawFillRectangle(45, 275, 229, 276, 0xFFFF);

    // Labels
    if (viewType == 0) {
        LCD_DrawStringRotated(0, 205, 0xFFFF, 0x0000, "Pressure (hPa)", 16, 1);
    } else if (viewType == 1) {
        LCD_DrawStringRotated(0, 200, 0xFFFF, 0x0000, "Humidity (%)", 16, 1);
    } else {
        LCD_DrawStringRotated(0, 180, 0xFFFF, 0x0000, "Temp (F)", 16, 1);
    }
    LCD_DrawString(91, 299, 0xFFFF, 0x0000, "Time (min)", 16, 1);

    // Ticks
    int yOffset = 6;
    int xOffset = 60;

    int32_t yMax = -100000;
    int32_t yMin = 100000;

    for (int i = 0; i < 10; i++) {
        if (loggedData[i][viewType] < yMin) {
            yMin = loggedData[i][viewType];
        }

        if (loggedData[i][viewType] > yMax) {
            yMax = loggedData[i][viewType];
        }

        if (predictedData[i][viewType] < yMin) {
            yMin = predictedData[i][viewType];
        }

        if (predictedData[i][viewType] > yMax) {
            yMax = predictedData[i][viewType];
        }
    }

    int32_t range = yMax - yMin;

    int32_t padding = range / 10;
    yMin -= padding;
    yMax += padding;

    range = yMax - yMin;
    
    int32_t tickInterval = range / 14;
    if (tickInterval == 0) {
        tickInterval = 1;
    }

    int32_t magnitude = 1;
    while (tickInterval > 10) {
        tickInterval /= 10;
        magnitude *= 10;
    }

    if (tickInterval <= 2) {
        tickInterval = 2;
    } else if (tickInterval <= 5) {
        tickInterval = 5;
    } else {
        tickInterval = 10;
    }

    tickInterval *= magnitude;

    yMin = (yMin / tickInterval) * tickInterval;
    yMax = ((yMax / tickInterval) + 1) * tickInterval;

    range = yMax - yMin;
    int32_t numTicks = range / tickInterval;

    if (numTicks != 14) {
        yMax = yMin + (14 * tickInterval);
    }

    if (ticks) {
        for (int i = 0; i <= 14; i++) {
            int tickY = yOffset + (i * 270 / 14);
            LCD_DrawFillRectangle(40, tickY, 47, tickY, 0xFFFF);

            char labelStr[16];
            sprintf(labelStr, "%d", yMax - (i * tickInterval));
            LCD_DrawString(20, tickY - yOffset, 0xFFFF, 0x0000, labelStr, 12, 0);
        }

        for (int i = 0; i <= 10; i++) {
            LCD_DrawFillRectangle(xOffset + (i * 17), 272, xOffset + (i * 17), 279, 0xFFFF);

            char labelStr[16];
            sprintf(labelStr, "%d", i + 1);
            LCD_DrawString(xOffset + (i * 17) - 2, 282, 0xFFFF, 0x0000, labelStr, 12, 0);
        }
    }

    // Data Points
    int xZero = 44;
    int yZero = 270;

    range = yMax - yMin;

    for (int i = 0; i < 9; i++) {
        int x1 = xZero + (i * 17);
        int y1 = yOffset + yZero - ((loggedData[i][viewType] - yMin) * yZero / range);
        int x2 = xZero + ((i + 1) * 17);
        int y2 = yOffset + yZero - ((loggedData[i + 1][viewType] - yMin) * yZero / range);
        LCD_DrawLine(x1, y1, x2, y2, 0xF800);

        int px1 = xZero + (i * 17);
        int py1 = yOffset + yZero - ((predictedData[i][viewType] - yMin) * yZero / range);
        int px2 = xZero + ((i + 1) * 17);
        int py2 = yOffset + yZero - ((predictedData[i + 1][viewType] - yMin) * yZero / range);
        LCD_DrawLine(px1, py1, px2, py2, 0x001F);
    }
}

void settings(uint8_t highlight, bool editing, int* asyncPollInterval, int* livePollInterval) {
    LCD_Clear(0x0000);

    const char* options[] = {"Async Polling Rate", "Live Polling Rate"};
    for (uint8_t i = 0; i < 2; i++) {
        if (i == 0) {
            char intervalStr[32];
            sprintf(intervalStr, "%d", *asyncPollInterval / 1000000);
            if (i == highlight) {
                if (editing) {
                    LCD_DrawRectangle(0, i * 40, 240, 40 + i * 40, 0x01CF);
                    LCD_DrawString(10, 12 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);

                     LCD_DrawFillRectangle(170, 4 + i * 40, 235, 36 + i * 40, 0x01CF);
                     LCD_DrawString(200, 12 + i * 40, 0xFFFF, 0x0000, intervalStr, 16, 1);
                } else {
                    LCD_DrawFillRectangle(0, i * 40, 240, 40 + i * 40, 0x01CF);
                    LCD_DrawString(10, 12 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);

                    LCD_DrawFillRectangle(170, 4 + i * 40, 235, 36 + i * 40, 0x0000);
                    LCD_DrawString(200, 12 + i * 40, 0xFFFF, 0x0000, intervalStr, 16, 1);
                }
            } else {
                LCD_DrawString(10, 12 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);
                LCD_DrawString(200, 12 + i * 40, 0xFFFF, 0x0000, intervalStr, 16, 0);
            }
        } else if (i == 1) {
            char intervalStr[32];
            sprintf(intervalStr, "%d", *livePollInterval / 1000000);
            if (i == highlight) {
                if (editing) {
                        LCD_DrawRectangle(0, i * 40, 240, 40 + i * 40, 0x01CF);
                        LCD_DrawString(10, 12 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);

                        LCD_DrawFillRectangle(170, 4 + i * 40, 235, 36 + i * 40, 0x01CF);
                        LCD_DrawString(200, 12 + i * 40, 0xFFFF, 0x0000, intervalStr, 16, 1);
                    } else {
                        LCD_DrawFillRectangle(0, i * 40, 240, 40 + i * 40, 0x01CF);
                        LCD_DrawString(10, 12 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);

                        LCD_DrawFillRectangle(170, 4 + i * 40, 235, 36 + i * 40, 0x0000);
                        LCD_DrawString(200, 12 + i * 40, 0xFFFF, 0x0000, intervalStr, 16, 1);
                    } 
            } else {
                LCD_DrawString(10, 12 + i * 40, 0xFFFF, 0x0000, options[i], 16, 1);
                LCD_DrawString(200, 12 + i * 40, 0xFFFF, 0x0000, intervalStr, 16, 0);
            }
        }
    }
}

void about(void) {
    LCD_Clear(0x0000);
    LCD_DrawString(10, 0, 0xFFFF, 0x0000, "Weather Station", 16, 1);
    LCD_DrawString(10, 20, 0xFFFF, 0x0000, "Channel 15", 16, 1);

    LCD_DrawString(10, 140, 0xFFFF, 0x0000, "Created by:", 16, 1);
    LCD_DrawString(10, 160, 0xFFFF, 0x0000, "Aayush Singh", 16, 1);
    LCD_DrawString(10, 180, 0xFFFF, 0x0000, "Mahdi El Husseini", 16, 1);
    LCD_DrawString(10, 200, 0xFFFF, 0x0000, "Nirrbhay Raghavan", 16, 1);
    LCD_DrawString(10, 220, 0xFFFF, 0x0000, "Quincy DeKlerk", 16, 1);

    LCD_DrawString(10, 300, 0xFFFF, 0x0000, "Version 1.0", 16, 1);
}