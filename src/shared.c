#include "pico/mutex.h"

static mutex_t spi1_mutex;

void spi1_init(void) {
    mutex_init(&spi1_mutex);
}

void spi1_lock(void) {
    mutex_enter_blocking(&spi1_mutex);
}

void spi1_unlock(void) {
    mutex_exit(&spi1_mutex);
}