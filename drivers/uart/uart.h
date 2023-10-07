#ifndef INC_UART_H
#define INC_UART_H

#include <stdint.h>
#include <stdbool.h>

void uart_setup(void);
void uart_teardown(void);
void uart_write(uint8_t* data, const uint32_t length);
void uart_write_byte(uint8_t data);
void uart_write_16(uint16_t data);
uint32_t uart_read(uint8_t* data, const uint32_t length);
uint8_t uart_read_byte(void);
void uart_print(const uint8_t* data);
void uart_println(const uint8_t* data);
bool uart_data_available(void);

#endif/* INC_UART_H */
