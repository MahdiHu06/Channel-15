#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"

////////////////////////////////////////////////////////////////////////

typedef struct { 
    uint16_t freq_hz;
    uint16_t dur_ms; 
} note_t;

static const note_t TEMP[] = { {880,150},{0,50},{660,150},{0,50},{587,150},{0,50},{523,300} };
static const note_t PRESSURE[]  = { {988,150},{0,50},{1319,150},{0,50},{1175,300} };
static const note_t HUMID[] = { {740,150},{0,50},{740,150},{0,50},{988,300} };

// 0=idle, 1=rain, 2=sun, 3=humid
static volatile int mode = 0;
static volatile int idx = 0;
static volatile int left_ms = 0;

static uint spk_slice, spk_chan;
static uint tick_slice;

typedef enum { AUDIO_IDLE=0, AUDIO_TEMP=1, AUDIO_PRESSURE=2, AUDIO_HUMID=3 } audio_mode_t;

// pending request written by other code (main or ISR)
static volatile int mode_req = -1;

static inline void audio_request(audio_mode_t m) {
    mode_req = (int)m;
}
static inline void audio_stop(void) {
    mode_req = (int)AUDIO_IDLE;
}

static inline void uart_puts_string(const char *s) {
    //just pasting a string to terminal to let us know what we are doing
    uart_puts(uart0, s);
    uart_puts(uart0, "\r\n");
}