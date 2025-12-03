#include "hardware/uart.h" 
#include "hardware/gpio.h" 
#include <stdio.h>  
#include <string.h>
#include "pico/stdlib.h"
#include "sdcard.h"

#define BUFSIZE 100
char serbuf[BUFSIZE];
int seridx = 0;
int newline_seen = 0;

// add this here so that compiler does not complain about implicit function
// in init_uart_irq
void uart_rx_handler();

/*******************************************************************/

void init_uart() {
    stdio_init_all();
    
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, false);
}

void init_uart_irq() {
    uart_set_fifo_enabled(uart0, false);
    uart_set_irq_enables(uart0, true, false);
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_handler);
    irq_set_enabled(UART0_IRQ, true);
}



void uart_rx_handler() {
    uart_get_hw(uart0)->icr = UART_UARTICR_RXIC_BITS;
    if(seridx >= BUFSIZE) {
        return;
    }
    char c = uart_get_hw(uart0)->dr;

    if(c == 0x0A){
        newline_seen = 1;
    }

    if(c == 8 && seridx > 0){
        uart_putc_raw(uart0, 0x08);
        uart_putc_raw(uart0, 0x20);
        uart_putc_raw(uart0, 0x08);
        seridx--;
        serbuf[seridx] = '\0';
        return;
    }
    if(c != 8){
        uart_putc_raw(uart0, c);
        serbuf[seridx] = c;
        seridx++;
    }
}


int _read(__unused int handle, char *buffer, int length) {
    while(!newline_seen){
        sleep_ms(5);
    }
    newline_seen = 0;
    int chars_to_copy = seridx;
    if(chars_to_copy >= length){
        chars_to_copy = length - 1;  // Leave room for null terminator
    }
    for(int i = 0; i < chars_to_copy; i++){
        buffer[i] = serbuf[i];
    }
    buffer[chars_to_copy] = '\0';  // Null terminate
    
    // Clear the serial buffer
    memset(serbuf, 0, BUFSIZE);
    seridx = 0;
    
    return chars_to_copy;
}

int _write(__unused int handle, char *buffer, int length) {
    uart_write_blocking(uart0, (const uint8_t*)buffer, length);
    return length;
}

/*******************************************************************/

struct commands_t {
    const char *cmd;
    void      (*fn)(int argc, char *argv[]);
};

struct commands_t cmds[] = {
        { "append", append },
        { "cat", cat },
        { "cd", cd },
        { "date", date },
        { "input", input },
        { "ls", ls },
        { "mkdir", mkdir },
        { "mount", mount },
        { "pwd", pwd },
        { "rm", rm },
        { "restart", restart }
};

// This function inserts a string into the input buffer and echoes it to the UART
// but whatever is "typed" by this function can be edited by the user.
void insert_echo_string(const char* str) {
    // Print the string and copy it into serbuf, allowing editing
    seridx = 0;
    newline_seen = 0;
    memset(serbuf, 0, BUFSIZE);

    // Print and fill serbuf with the initial string
    for (int i = 0; str[i] != '\0' && seridx < BUFSIZE - 1; i++) {
        char c = str[i];
        uart_write_blocking(uart0, (uint8_t*)&c, 1);
        serbuf[seridx++] = c;
    }
}

void parse_command(const char* input_const) {
    char input_copy[128];
    strncpy(input_copy, input_const, sizeof(input_copy) - 1);
    input_copy[sizeof(input_copy) - 1] = '\0';
    
    char *token = strtok(input_copy, " ");
    int argc = 0;
    char *argv[10];
    while (token != NULL && argc < 10) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }
    
    int i = 0;
    for(; i<sizeof cmds/sizeof cmds[0]; i++) {
        if (strcmp(cmds[i].cmd, argv[0]) == 0) {
            cmds[i].fn(argc, argv);
            break;
        }
    }
    if (i == (sizeof cmds/sizeof cmds[0])) {
        printf("Unknown command: %s\n", argv[0]);
    }
}

void command_shell() {
    char input[100];


    // Disable buffering for stdout
    setbuf(stdout, NULL);
    
    printf("\n\nSD Card Command Shell\n");
    printf("Type 'mount' to mount the SD card.\n");


    
    while (1) {
        // Clear input buffer
        memset(input, 0, sizeof(input));
        
        printf("\n> ");
        fflush(stdout);
        
        // Read command
        if (fgets(input, sizeof(input), stdin) == NULL) {
            continue;
        }
        
        // Remove trailing whitespace
        int len = strlen(input);
        while (len > 0 && (input[len-1] == '\n' || input[len-1] == '\r' || input[len-1] == ' ')) {
            input[--len] = '\0';
        }
        
        // Skip empty commands
        if (len == 0) {
            continue;
        }
        

        // Parse and execute
        parse_command(input);
    }
}