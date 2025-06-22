#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <stdio.h>

void uart_init();
void jetson_uart_task(void *param);

#endif