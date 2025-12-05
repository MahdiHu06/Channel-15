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
#include "../include/radio.h"
#include "../include/speaker.h"

#define BUFSIZE 32
char serbuf[BUFSIZE];
int seridx = 0;
int newline_seen = 0;

// add this here so that compiler does not complain about implicit function
void uart_rx_handler();

void init_uart() { 
    uart_init(uart0, 115200); 
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
}
void init_uart_irq() {
    // disable FIFO
    uart_set_fifo_enabled(uart0, false);

    // mask the interrupt
    uart0_hw->imsc = (1 << 4); 
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_handler);
    
    irq_set_enabled(UART0_IRQ, true);
}

void uart_rx_handler() {
    // Acknowledge Interrupt
    uart0_hw->icr = 1 << 4;

    if (seridx == BUFSIZE) { return; }
    
    char c = uart0_hw->dr;      // get character

    // check for what the character is
    if (c == 0x0A) { newline_seen = 1; }
    if ((c == 0x8) && (seridx > 0)) {
        putchar(c);     // backspace
        putchar(0x20);  // space
        putchar(c);     // backspace
        seridx--;   
        serbuf[seridx] = '\0';
        return;
    }

    // Otherwise
    serbuf[seridx] = c;
    uart_write_blocking(uart0, &serbuf[seridx], 1);
    seridx++;
}

int _read(__unused int handle, char *buffer, int length) {
    while (newline_seen == 0) { sleep_ms(5); }
    newline_seen = 0;
    for (int i = 0; i < seridx; i++) {
        if (i == length) { break; }
        buffer[i] = serbuf[i];
    }
    uint idx = seridx;
    seridx = 0;
    return idx;
}

int _write(__unused int handle, char *buffer, int length) {
    for (int i = 0; i < length; i++) { uart_write_blocking(uart0, &buffer[i], 1); }
    return length;
}

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

static void set_pwm_tone(uint32_t freq_hz, uint32_t duty_percent) {
    if (freq_hz == 0 || duty_percent == 0) {
        pwm_set_chan_level(spk_slice, spk_chan, 0);
        return;
    }

    //setting clock frequency for pwm
    uint32_t top = 1000000u / freq_hz;
    if (top == 0) {
        top = 1;
    }
    top -= 1;
    if (top > 0xFFFF) {
        top = 0xFFFF;
    }

    pwm_hw->slice[spk_slice].top = top;
    //all for pwm frequency
    uint32_t period = top + 1;
    uint32_t level = (period * duty_percent) / 100u;
    if (level > period) {
        level = period;
    }
    pwm_set_chan_level(spk_slice, spk_chan, level);
}

//activated to tick the pwm based on mood, have to check this in lab
static void pwm_tick_irq(void) {
    pwm_clear_irq(tick_slice);

    // apply any pending request immediately
    int pending = mode_req;
    if (pending >= 0) {
        mode_req = -1;
        if (pending < 0 || pending > 3) {
            pending = 0;
        }
        mode = pending;
        idx = 0;
        left_ms = 0;
        if (mode == 0) {
            pwm_set_chan_level(spk_slice, spk_chan, 0);
        }
    }

    if (mode == 0) {
        return;
    }

    const note_t *sequence;
    int length;

    if (mode == 1) {
        sequence = TEMP;
        length = (int)(sizeof(TEMP) / sizeof(TEMP[0]));
    }
    else if (mode == 2) {
        sequence = PRESSURE;
        length = (int)(sizeof(PRESSURE) / sizeof(PRESSURE[0]));
    }
    else if (mode == 3) {
        sequence = HUMID;
        length = (int)(sizeof(HUMID) / sizeof(HUMID[0]));
    }
    else {
        mode = 0;
        return;
    }

    if (left_ms == 0) {
        if (idx >= length) {
            pwm_set_chan_level(spk_slice, spk_chan, 0);
            mode = 0;
            idx = 0;
            return;
        }

        uint16_t frequency = sequence[idx].freq_hz;
        uint16_t duration = sequence[idx].dur_ms;
        idx++;

        if (frequency == 0) {
            pwm_set_chan_level(spk_slice, spk_chan, 0);
        }
        else {
            set_pwm_tone(frequency, 50);
        }

        left_ms = duration;
    }
    else {
        left_ms--;
    }
}

void speakerInit(void) {
    init_uart();
    init_uart_irq();

    setbuf(stdout, NULL); // disable buffering for stdout

    gpio_set_function(PWM_PIN_A, GPIO_FUNC_PWM);
    spk_slice = pwm_gpio_to_slice_num(PWM_PIN_A);
    spk_chan  = pwm_gpio_to_channel(PWM_PIN_A);
    pwm_hw->slice[spk_slice].csr &= ~PWM_CH0_CSR_DIVMODE_BITS;
    // 125MHz/125=1MHz
    pwm_hw->slice[spk_slice].div = (125 << PWM_CH0_DIV_INT_LSB);
    pwm_hw->slice[spk_slice].top = 1000 - 1;
    pwm_set_chan_level(spk_slice, spk_chan, 0);
    pwm_set_enabled(spk_slice, true);

    gpio_set_function(PWM_PIN_B, GPIO_FUNC_PWM);
    tick_slice = pwm_gpio_to_slice_num(PWM_PIN_B);
    pwm_set_clkdiv(tick_slice, 125);
    pwm_set_wrap(tick_slice, 999);
    pwm_clear_irq(tick_slice);
    pwm_set_irq_enabled(tick_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP_0, pwm_tick_irq);
    irq_set_enabled(PWM_IRQ_WRAP_0, true);
    pwm_set_enabled(tick_slice, true);

    uart_puts_string("READY TO READ...");

    audio_mode_t last = AUDIO_IDLE;
}

void speakerLoop(void) {
    speakerInit();

    uart_puts_string("STARTING...");

    audio_mode_t last = AUDIO_IDLE;

    for (;;) {
        uint8_t request_buf[1];
        request_buf[0] = 0x07; // Request all data

        if (!sendDataReliable(RADIO_SPI_CSN_PIN, RADIO_SPI_CSN_PIN, request_buf, 1)) {
            printf("Failed to send request\n");
            continue; 
        }

        startRadioReceive(RADIO_SPI_CSN_PIN);

        uint8_t response_buf[64];
        uint8_t response_len;
        
        if (!receivePacketRaw(RADIO_SPI_CSN_PIN, response_buf, &response_len, 500)) {
            printf("No response from sensor\n");
            continue;
        }

        printf("Got response: len=%d, type=0x%02X\n", response_len, response_buf[0]);

        if (response_len < 2 || response_buf[0] != PKT_TYPE_DATA) {
            printf("Invalid response packet type: 0x%02X\n", response_buf[0]);
            continue;
        }

        uint8_t *payload = &response_buf[2];
        uint8_t payload_len = response_len - 2;

        printf("Payload len=%d, request_type=0x%02X\n", payload_len, payload[0]);

        if (payload_len != 13 || payload[0] != 0x07) {
            printf("Unexpected payload: len=%d, type=0x%02X\n", payload_len, payload[0]);
            continue;
        }

        float temp = 0;
        float pressure = 0;
        float humidity = 0;
        
        memcpy(&temp,     &payload[1],  sizeof(float));
        memcpy(&pressure, &payload[5],  sizeof(float));
        memcpy(&humidity, &payload[9],  sizeof(float));

        printf("Temp: %.2f, Pressure: %.2f, Humidity: %.2f\n", temp, pressure, humidity);

        audio_mode_t want = AUDIO_IDLE;

        if (humidity >= HUMIDITY_RH_THRESHOLD_HIGH) {
            want = AUDIO_HUMID;
        } else if (pressure <= PRESSURE_KPA_THRESHOLD_LOW) {
            want = AUDIO_PRESSURE;
        } else if (temp >= TEMP_C_THRESHOLD_HIGH) {
            want = AUDIO_TEMP;
        } else {
            want = AUDIO_IDLE;
        }

        if (want != last) {
            last = want;

            if (want == AUDIO_TEMP) {
                uart_puts_string("\n!!!ALERT: TEMP HAS EXCEEDED 90 DEGREES FARENHEIT!!!\n");
                audio_request(AUDIO_TEMP);
            } else if (want == AUDIO_PRESSURE) {
                uart_puts_string("\n!!!ALERT: PRESSURE HAS REACHED LEVELS SIGNALING PRECIPITATION!!!\n");
                audio_request(AUDIO_PRESSURE);
            } else if (want == AUDIO_HUMID) {
                uart_puts_string("\n!!!ALERT: HUMIDITY HAS PASSED 60 PERCENT!!!\n");
                audio_request(AUDIO_HUMID);
            } else {
                uart_puts_string("ALERT: IDLE");
                audio_stop();
            }
        }
        sleep_ms(2000);
   }
}