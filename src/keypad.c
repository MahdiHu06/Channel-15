#include "pico/stdlib.h"
#include <hardware/gpio.h>
#include <stdio.h>
#include "queue.h"

// Global column variable
int col = -1;

// Global key state
static bool state[16]; // Are keys pressed/released

// Keymap for the keypad
const char keymap[17] = "DCBA#9630852*741";

void keypad_drive_column();
void keypad_isr();

/********************************************************* */
// Implement the functions below.

void keypad_init_pins() {
    /* Enable Inputs Start */
    gpio_init_mask((1u << 2) | (1u << 3) | (1u << 4) | (1u << 5));
    gpio_set_dir(2, false);
    gpio_set_dir(3, false);
    gpio_set_dir(4, false);
    gpio_set_dir(5, false);
    /* Enable Inputs End */


    /* Enable Outputs Start */

    // Set Enabled Pins to Low
    sio_hw->gpio_clr = (1ul << 6) | (1ul << 7) | (1ul << 8) | (1ul << 9);

    // Set input enable on, output disable off
    *(io_rw_32 *) hw_xor_alias_untyped((volatile void *) &pads_bank0_hw->io[6]) = (6 ^ PADS_BANK0_GPIO0_IE_BITS) & (PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    *(io_rw_32 *) hw_xor_alias_untyped((volatile void *) &pads_bank0_hw->io[7]) = (7 ^ PADS_BANK0_GPIO0_IE_BITS) & (PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    *(io_rw_32 *) hw_xor_alias_untyped((volatile void *) &pads_bank0_hw->io[8]) = (8 ^ PADS_BANK0_GPIO0_IE_BITS) & (PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    *(io_rw_32 *) hw_xor_alias_untyped((volatile void *) &pads_bank0_hw->io[9]) = (9 ^ PADS_BANK0_GPIO0_IE_BITS) & (PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);

    // Zero fields
    io_bank0_hw->io[6].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    io_bank0_hw->io[7].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    io_bank0_hw->io[8].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    io_bank0_hw->io[9].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;

    // Remove pad iso
    *(io_rw_32 *) hw_clear_alias_untyped((volatile void *) &pads_bank0_hw->io[6]) = PADS_BANK0_GPIO0_ISO_BITS;
    *(io_rw_32 *) hw_clear_alias_untyped((volatile void *) &pads_bank0_hw->io[7]) = PADS_BANK0_GPIO0_ISO_BITS;
    *(io_rw_32 *) hw_clear_alias_untyped((volatile void *) &pads_bank0_hw->io[8]) = PADS_BANK0_GPIO0_ISO_BITS;
    *(io_rw_32 *) hw_clear_alias_untyped((volatile void *) &pads_bank0_hw->io[9]) = PADS_BANK0_GPIO0_ISO_BITS;
    
    // Enable Output
    sio_hw->gpio_oe_set = (1ul << 6) | (1ul << 7) | (1ul << 8) | (1ul << 9);

    /* Enable Outputs End */
}

void keypad_init_timer() {
    // Enable Alarms
    hw_set_bits(&timer0_hw->inte, (1u << 0) | (1u << 1));

    // Alarm 0
    irq_set_exclusive_handler(timer_hardware_alarm_get_irq_num(timer0_hw, 0), keypad_drive_column);
    irq_set_enabled(timer_hardware_alarm_get_irq_num(timer0_hw, 0), true);
    timer0_hw->alarm[0] = (uint32_t) timer0_hw -> timerawl + (1000000 * 1);

    // Alarm 1
    irq_set_exclusive_handler(timer_hardware_alarm_get_irq_num(timer0_hw, 1), keypad_isr);
    irq_set_enabled(timer_hardware_alarm_get_irq_num(timer0_hw, 1), true);
    timer0_hw->alarm[1] = (uint32_t) timer0_hw -> timerawl + (1000000 * 1.1);
}

void keypad_drive_column() {
    hw_clear_bits(&timer0_hw->intr, (1u << 0));

    col++;

    if (col == 4) {
        col = 0;
    }

    if (col == 0) {
        sio_hw -> gpio_clr = (1u << 9) | (1u << 8) | (1u << 7) | (1u << 6);
        sio_hw -> gpio_togl = (1u << (col + 6));
    } else {
        sio_hw -> gpio_togl =  (1u << (col + 6)) | (1u << (col + 5));
    }

    timer0_hw->alarm[0] = (uint32_t) timer0_hw -> timerawl + (1000000 * 0.0025);
}

uint8_t keypad_read_rows() {
    return (sio_hw -> gpio_in >> 2) & 0b1111;
}

void keypad_isr() {
    hw_clear_bits(&timer0_hw->intr, (1u << 1));

    uint8_t currRows = keypad_read_rows();

    for (int r = 0; r < 4; r++) {
        if (currRows & (1u << r)) {
            if (state[r + (4 * col)] == 0) {
                state[r + (4 * col)] = 1;

                key_push(((1u << 8) | keymap[r + (4 * col)]));
            }
        } else {
            if (state[r + (4 * col)] == 1) {
                state[r + (4 * col)] = 0;

                key_push(((0u << 8) | keymap[r + (4 * col)]));
            }
        }
    }

    timer0_hw->alarm[1] = (uint32_t) timer0_hw->timerawl + (1000000 * 0.0025);
}
