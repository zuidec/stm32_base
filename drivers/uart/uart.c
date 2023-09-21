#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "core/uart.h"
#include "core/fifo.h"

#define BAUD_RATE           (115200)
#define FIFO_BUFFER_SIZE    (128)    // Needs to be a power of 2 in order for the mask to work

static uint8_t data_buffer[FIFO_BUFFER_SIZE] = {0U};
static fifo_buffer_t fifo = {0U};

void usart2_isr(void)   {

    // Reset interrupt flags
    const bool overrun_occurred = usart_get_flag(USART2, USART_FLAG_ORE)==1;
    const bool received_data = usart_get_flag(USART2, USART_FLAG_RXNE) ==1;

    // Catch the data then write it to the fifo buffer
    if(received_data||overrun_occurred) {
        if(!fifo_buffer_write(&fifo, (uint8_t)usart_recv(USART2)))  {
            // Handle write failure
        }
    }
}

void uart_setup(void)   {
    //Start the clock for the UART
    rcc_periph_clock_enable(RCC_USART2);

    // Set up the FIFO buffer
    fifo_buffer_setup(&fifo, data_buffer, FIFO_BUFFER_SIZE);

    // Set the data to 8+1 no parity, at the defined baud rate
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART2, 8);
    usart_set_baudrate(USART2, BAUD_RATE);
    usart_set_parity(USART2,0);
    usart_set_stopbits(USART2,1);

    // Set uart mode to TX/RX
    usart_set_mode(USART2, USART_MODE_TX_RX);
    
    // Enable RX interrupt
    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);

    // Turn on the UART
    usart_enable(USART2);


}

void uart_teardown(void)   {
 
    // Disable RX interrupt
    usart_disable_rx_interrupt(USART2);
    
    // Turn off the UART
    usart_disable(USART2);
    nvic_disable_irq(NVIC_USART2_IRQ);

    //Stop the clock for the UART
    rcc_periph_clock_disable(RCC_USART2);

}

void uart_write(uint8_t* data, const uint32_t length)   {

    // Loop through data and send a byte at a time
    for( uint32_t i=0; i< length; i++)  {
        uart_write_byte(data[i]);
    }
}

void uart_write_byte(uint8_t data)  {
    usart_send_blocking(USART2,(uint16_t)data);
}

uint32_t uart_read(uint8_t* data, const uint32_t length)    {
   
    if(length == 0 )    {

        return 0;  
    }
    
    for(uint32_t bytes_read = 0; bytes_read < length; bytes_read++)    {
        if(!fifo_buffer_read(&fifo, &data[bytes_read]))  {
            return bytes_read;
        }
    }

    return length;
}

uint8_t uart_read_byte(void)    {

    uint8_t byte = 0;
    (void)uart_read(&byte, 1);

    return byte;
}

bool uart_data_available(void)  {
    return !fifo_buffer_empty(&fifo);
}