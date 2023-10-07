#ifndef INC_I2C_H
#define INC_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "drivers/fifo.h"



typedef struct i2c_handle_t   {
    uint32_t i2c_base_address;
    uint32_t gpio_port;
    uint32_t sda_pin;
    uint32_t scl_pin;
    volatile uint32_t* CR1;
    volatile uint32_t* SR1;
    volatile uint32_t* SR2;
    volatile uint32_t* DR;
    volatile fifo_buffer_t *fifo;
      
}i2c_handle_t;

void i2c_setup(i2c_handle_t* i2c, const uint32_t i2c_address, const uint32_t i2c_gpio_port, const uint32_t sda_pin, const uint32_t scl_pin, fifo_buffer_t* fifo);
void i2c_teardown(i2c_handle_t* i2c);
uint32_t i2c_read_bytes(i2c_handle_t* i2c, uint8_t* data, uint32_t bytes_to_read, uint8_t device_address);
uint8_t i2c_read_byte(i2c_handle_t* i2c, uint8_t device_address, uint8_t device_register);
void i2c_write_bytes(i2c_handle_t* i2c, uint8_t* data, uint32_t length, uint8_t device_address);
void i2c_write_byte(i2c_handle_t* i2c, uint8_t byte, uint8_t device_address, uint8_t device_register);
bool i2c_data_available(i2c_handle_t* i2c);

#endif/* INC_I2C_H */
