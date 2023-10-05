#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "i2c.h"

fifo_buffer_t* i2c1_isr_fifo;
fifo_buffer_t* i2c2_isr_fifo;
fifo_buffer_t* i2c3_isr_fifo;


void i2c1_isr(void) {
    // Reset interrupt flags


    // Catch the data then write it to the fifo buffer
    // Not sure if this will work passing the address of the fifo as a uint32_t
    if(!fifo_buffer_write(i2c1_isr_fifo, (uint8_t)i2c_read_byte))  {
        // Handle write failure
    }
}

void i2c_setup(i2c_handle_t* i2c, const uint32_t i2c_address, const uint32_t i2c_gpio_port, const uint32_t sda_pin, const uint32_t scl_pin)   {
    /*
        The following is the required sequence in master mode. 
        • Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
            Minimum 4MHz fast mode and 2MHz slow mode
        • Configure the clock control registers 
        • Configure the rise time register
        • Program the I2C_CR1 register to enable the peripheral
        • Set the START bit in the I2C_CR1 register to generate a Start condition
    */
    
    switch(i2c_address) {
        case I2C1:  {
            // Pass values to i2c_handle to initialize
            i2c->i2c_base_address = i2c_address;
            i2c->gpio_port = i2c_gpio_port;
            i2c->sda_pin = sda_pin;
            i2c->scl_pin = scl_pin;
            i2c1_isr_fifo = i2c->fifo;

            // Enable GPIO and set alternate function for i2c: all I2C1 are on port B
            rcc_periph_clock_enable(RCC_GPIOB);
            gpio_mode_setup(i2c_gpio_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, sda_pin | scl_pin);
            gpio_set_af(i2c_gpio_port, GPIO_AF4, sda_pin | scl_pin);
            
            // Start peripheral clock for i2c
            rcc_periph_clock_enable(RCC_I2C1);

            // Reset i2c
            I2C1_CR1 |= BIT15;
            I2C1_CR1 &= ~BIT15;

            // Set CR2 bit9 to enable event interrupt and set freq to 45mhz
            I2C1_CR2 = BIT9 | 45;

            // Set CCR register, see reference manual for calculation. bit 15 sets full speed
            I2C1_CCR = (36<<0) | BIT15;

            // Configure trise, see reference manual for calculation
            I2C1_TRISE = 14;

            // Enable i2c
            i2c_peripheral_enable(I2C1);

            
            break;
        }
        case I2C2:  {
            i2c->i2c_base_address = i2c_address;
            i2c2_isr_fifo = i2c->fifo;

            break;
        }
        case I2C3:  {
            i2c->i2c_base_address = i2c_address;
            i2c3_isr_fifo = i2c->fifo;

            break;
        }
    }
    
}

void i2c_teardown(i2c_handle_t* i2c)    {

}

uint32_t i2c_read_bytes(i2c_handle_t* i2c, uint8_t* data, uint32_t bytes_to_read, uint8_t device_address)   {

}

uint8_t i2c_read_byte(i2c_handle_t* i2c, uint8_t device_address, uint8_t device_register)    {
    i2c_send_start(i2c->i2c_base_address);
    i2c_send_7bit_address(i2c->i2c_base_address,device_address, I2C_READ);
    i2c_send_data(i2c->i2c_base_address, device_register);
    i2c_get_data(i2c->i2c_base_address);
    i2c_send_stop(i2c->i2c_base_address);
}

void i2c_write_bytes(i2c_handle_t* i2c, uint8_t* data, uint32_t length, uint8_t device_address) {

}

void i2c_write_byte(i2c_handle_t* i2c, uint8_t byte, uint8_t device_address)    {

}

bool i2c_data_available(i2c_handle_t* i2c)  {

}
