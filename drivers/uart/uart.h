#ifndef INC_UART_H
#define INC_UART_H

#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdbool.h>

#include "drivers/fifo.h"

typedef struct uart_t   {
        uint32_t uart_base;
        uint32_t baudrate;
        fifo_buffer_t* fifo;
} uart_t;

void uart_setup(uart_t* uart);
void uart_teardown(const uart_t* uart);
void uart_write(const uart_t* uart, uint8_t* data, const uint32_t length);
void uart_write_byte(const uart_t* uart, uint8_t data);
void uart_write_16(const uart_t* uart, uint16_t data);
uint32_t uart_read(const uart_t* uart, uint8_t* data, const uint32_t length);
uint8_t uart_read_byte(const uart_t* uart);
void uart_print(const uart_t* uart, const uint8_t* data);
void uart_println(const uart_t* uart, const uint8_t* data);
bool uart_data_available(const uart_t* uart);

#endif/* INC_UART_H */
