#include "../include/speaker.h"
#include "../include/radio.h"

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

////////////////////////////////////////////////////////////////////////

// TODO: tune thresholds values so that they reflect what we want
#define TEMP_F_THRESHOLD_HIGH      90.0f //90 degrees farenheit
#define PRESSURE_KPA_THRESHOLD_LOW 100.9f //signals potential precipitation
#define HUMIDITY_RH_THRESHOLD_HIGH 60.0f //level that indicates humidity is high



///////////////////////////////////////////////////////////////////////
 int speaker_run() {
    for (;;) {
        uint8_t request_buf[1];
        request_buf[0] = 0x07; // Request all data

        sendDataReliable(RADIO_SPI_CSN_PIN, RADIO_SPI_CSN_PIN, request_buf, 1);

        uint8_t response_buf[13];
        uint8_t response_len;
        receivePacketRaw_blocking(RADIO_SPI_CSN_PIN, response_buf, &response_len);
        float pressure_kpa = 0;
        float temp_f = 0;
        float humidity_rh = 0;
        if (response_len == 13 && response_buf[0] == 0x07) {
            memcpy(&temp_f,     &response_buf[1],  sizeof(float));
            memcpy(&pressure_kpa, &response_buf[5],  sizeof(float));
            memcpy(&humidity_rh, &response_buf[9],  sizeof(float));
        }

        audio_mode_t want = AUDIO_IDLE;

        if (humidity_rh >= HUMIDITY_RH_THRESHOLD_HIGH) {
            want = AUDIO_HUMID;
        } else if (pressure_kpa <= PRESSURE_KPA_THRESHOLD_LOW) {
            want = AUDIO_PRESSURE;
        } else if (temp_c >= TEMP_F_THRESHOLD_HIGH) {
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
        sleep_ms(1000);
    }
    return 0;
}