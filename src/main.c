#include "main.h"
#include "uart.h"
#include "i2c.h"
////////////////////////////////////////////////////////////////////////

// Copy global variables, init_uart_irq, uart_rx_handler, _read, and _write from STEP4.
#define BUFSIZE 32
char serbuf[BUFSIZE];
int seridx = 0;
int newline_seen = 0;

/* Uart Initilization */
void init_uart(){
    uart_init(uart0, 115200);
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
}

void init_uart_irq() {
    // disable FIFO
    uart_set_fifo_enabled(uart0, false);
    uart0_hw->imsc = (1 << 4); 
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_handler);
    irq_set_enabled(UART0_IRQ, true);   // Set to NVIC
}

void uart_rx_handler() {
    uart0_hw->icr = 1 << 4; // Acknowledge Interrupt
    // Don't exceed buffer size
    if (seridx == BUFSIZE) { return; }

    // check for what the character is
    char c = uart0_hw->dr;      // get character
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

/* Copied over from lab 7 */
void cmd_gpio(int argc, char **argv) {
    if (argc < 2) {
        printf("argc needs to be at least 2.\n");    // I'm unsure of what this line should be
        return;
    }

    if (strcmp(argv[1], "out") == 0) {
        if (argc != 3) {
            printf("argc needs to be equal to 3.\n");
            return;
        }
        int pinNum = atoi(argv[2]);
        if ((pinNum < 0) || pinNum > 47) {
            printf("Invalid pin number: %d. Must be between 0 and 47.\n", pinNum);
            return;
        }
        gpio_init(pinNum);
        gpio_set_dir(pinNum, true);
        printf("Initialized pin %d as an output.\n", pinNum);
        return;
    }
    if (strcmp(argv[1], "set") == 0) {
        if (argc != 4) {
            printf("argc needs to be equal to 4.\n");
            return;
        }
        int pinNum2 = atoi(argv[2]);
        if ((pinNum2 < 0) || (pinNum2 > 47)) {
            printf("Invalid pin number: %d. Must be between 0 and 47.\n", pinNum2);
            return;
        }
        if ((gpio_get_dir(pinNum2) == 0)) {
            return;
        }
        int pinValue = atoi(argv[3]);
        if ((pinValue != 0) && (pinValue != 1)) {
            printf("Error. Invalid argument.\n");
            return;
        }
        gpio_put(pinNum2, pinValue);
        printf("Set pin %d to %d\n", pinNum2, pinValue);
        return;
    }
    printf("Unknown command: \n");
}

/* main */
int main() {
    init_uart();
    init_uart_irq();

    setbuf(stdout, NULL); // Disable buffering for stdout

    // Welcome message
    printf("Mahdi's Peripheral Command Shell (PCS)\n");
    printf("Enter a command below.\n\n");

    // Inputs
    int argc;
    char *argv[10];
    char input[100];

    for (;;) {
        printf("\r\n>");
        fgets(input, sizeof(input), stdin);
        
        // This is needed
        fflush(stdin);
        
        // replace new line with null
        size_t j = strcspn(input, '\n');
        input[j] = '\0';    // look into this
        
        // tokenize input string
        char *token = strtok(input, " ");
        for (argc = 0; argc < 10; argc++) { 
            if (token == NULL) { break; }
            argv[argc] = token;
            token = strtok(NULL, " ");
        }

        // handle command
        cmd_gpio(argc, argv);
    }

    return EXIT_SUCCESS;    // should never hit
}