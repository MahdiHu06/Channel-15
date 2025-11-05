#ifndef UART_H
#define UART_H

#include "main.h"
#include "hardware/uart.h"

void init_uart();
void init_uart_irq();
void uart_rx_handler();

#endif