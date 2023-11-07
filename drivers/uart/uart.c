#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "uart.h"
#include "drivers/fifo.h"

#define USART_DATA_BITS_8     (8)

static fifo_buffer_t* usart1_fifo;
static fifo_buffer_t* usart2_fifo;
static fifo_buffer_t* usart3_fifo;

void usart1_isr(void)   {

    // Reset interrupt flags
    const bool overrun_occurred = usart_get_flag(USART1, USART_FLAG_ORE)==1;
    const bool received_data = usart_get_flag(USART1, USART_FLAG_RXNE) ==1;

    // Catch the data then write it to the fifo buffer
    if(received_data||overrun_occurred) {
        if(!fifo_buffer_write(usart1_fifo, (uint8_t)usart_recv(USART1)))  {
            // Handle write failure
        }
    }
}

void usart2_isr(void)   {

    // Reset interrupt flags
    const bool overrun_occurred = usart_get_flag(USART2, USART_FLAG_ORE)==1;
    const bool received_data = usart_get_flag(USART2, USART_FLAG_RXNE) ==1;

    // Catch the data then write it to the fifo buffer
    if(received_data||overrun_occurred) {
        if(!fifo_buffer_write(usart2_fifo, (uint8_t)usart_recv(USART2)))  {
            // Handle write failure
        }
    }
}

void usart3_isr(void)   {

    // Reset interrupt flags
    const bool overrun_occurred = usart_get_flag(USART3, USART_FLAG_ORE)==1;
    const bool received_data = usart_get_flag(USART3, USART_FLAG_RXNE) ==1;

    // Catch the data then write it to the fifo buffer
    if(received_data||overrun_occurred) {
        if(!fifo_buffer_write(usart3_fifo, (uint8_t)usart_recv(USART3)))  {
            // Handle write failure
        }
    }
}

void uart_setup(uart_t* uart)   {

    if(uart->uart_base==USART1) {
            //Start the clock for the UART
            rcc_periph_clock_enable(RCC_USART1);
            usart1_fifo = uart->fifo;
    }

    else if(uart->uart_base==USART2) {
            //Start the clock for the UART
            rcc_periph_clock_enable(RCC_USART2);
            usart2_fifo = uart->fifo;
    }

    else if(uart->uart_base==USART3) {
            //Start the clock for the UART
            rcc_periph_clock_enable(RCC_USART3);
            usart3_fifo = uart->fifo;
    }

    // Set the data to 8+1 no parity, at the defined baud rate
    usart_set_flow_control(uart->uart_base, USART_FLOWCONTROL_NONE);
    usart_set_databits(uart->uart_base, USART_DATA_BITS_8);
    usart_set_baudrate(uart->uart_base, uart->baudrate);
    usart_set_parity(uart->uart_base,USART_PARITY_NONE);
    usart_set_stopbits(uart->uart_base,USART_STOPBITS_1);

    // Set uart mode to TX/RX
    usart_set_mode(uart->uart_base, USART_MODE_TX_RX);
    
    // Enable RX interrupt
    usart_enable_rx_interrupt(uart->uart_base);
    if(uart->uart_base==USART1) {
            nvic_enable_irq(NVIC_USART1_IRQ);
    }

    else if(uart->uart_base==USART2) {
            nvic_enable_irq(NVIC_USART2_IRQ);
    }

    else if(uart->uart_base==USART3) {
            nvic_enable_irq(NVIC_USART3_IRQ);
    }
    // Turn on the UART
    usart_enable(uart->uart_base);


}

void uart_teardown(const uart_t* uart)   {
 
    // Disable RX interrupt
    usart_disable_rx_interrupt(uart->uart_base);
    
    // Disable interrupt handler
    if(uart->uart_base==USART1) {
            nvic_disable_irq(NVIC_USART1_IRQ);
    }

    else if(uart->uart_base==USART2) {
            nvic_disable_irq(NVIC_USART2_IRQ);
    }

    else if(uart->uart_base==USART3) {
            nvic_disable_irq(NVIC_USART3_IRQ);
    }

    //Stop the clock for the UART
    if(uart->uart_base==USART1) {
            rcc_periph_clock_disable(RCC_USART1);
    }

    else if(uart->uart_base==USART2) {
            rcc_periph_clock_disable(RCC_USART2);
    }

    else if(uart->uart_base==USART3) {
            rcc_periph_clock_disable(RCC_USART3);
    }

    // Turn off the UART
    usart_disable(uart->uart_base);


}

void uart_write(const uart_t* uart, uint8_t* data, const uint32_t length)   {

    // Loop through data and send a byte at a time
    for( uint32_t i=0; i< length; i++)  {
        uart_write_byte(uart, data[i]);
    }
}

void uart_write_16(const uart_t* uart, uint16_t data)   {

    // Loop through data and send a byte at a time
    usart_send_blocking(uart->uart_base,data);
}

void uart_write_byte(const uart_t* uart, uint8_t data)  {
    usart_send_blocking(uart->uart_base,(uint16_t)data);
}

uint32_t uart_read(const uart_t* uart, uint8_t* data, const uint32_t length)    {
   
    if(length == 0 )    {

        return 0;  
    }
    
    for(uint32_t bytes_read = 0; bytes_read < length; bytes_read++)    {
        if(!fifo_buffer_read(uart->fifo, &data[bytes_read]))  {
            return bytes_read;
        }
    }

    return length;
}

uint8_t uart_read_byte(const uart_t* uart)    {

    uint8_t byte = 0;
    (void)uart_read(uart, &byte, 1);

    return byte;
}

bool uart_data_available(const uart_t* uart)  {
    return !fifo_buffer_empty(uart->fifo);
}

void uart_print(const uart_t* uart, const uint8_t* data)   {
    uint8_t tx_buffer[strlen((char*)data)];
    sprintf((char*)tx_buffer, "%s", data);
    uart_write(uart, tx_buffer, strlen((char*)tx_buffer));
}

void uart_println(const uart_t* uart, const uint8_t* data) {
    uint8_t tx_buffer[strlen((char*)data)];
    sprintf((char*)tx_buffer, "%s\n", data);
    uart_write(uart, tx_buffer, strlen((char*)tx_buffer));
}
