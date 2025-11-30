#include "pico/stdlib.h"

int32_t randomFloat(int32_t min, int32_t max) {
    return min + (rand() % (max - min + 1));
}

void generateRandomArray(int32_t result[13][3], int32_t min, int32_t max) {
    for (int i = 0; i < 13; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = randomFloat(min, max);
        }
    }
}