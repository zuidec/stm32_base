#ifndef INC_I2C_H
#define INC_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "drivers/fifo.h"

#define HZ_TO_MHZ   (1000000U)
#define S_TO_NS     (1000000000U)

#ifdef STM32F4
    #define I2C_SM_T_LOW            (300U + 4700U)
    #define I2C_SM_MAX_SCL_RISE     (1000U)

    #define I2C_FM_T_LOW            (300U + 1300U)
    #define I2C_FM_MAX_SCL_RISE     (300U)    
#endif

typedef struct i2c_handle_t   {
    uint32_t i2c_base_address;
    uint32_t gpio_port;
    uint32_t sda_pin;
    uint32_t scl_pin;
     
}i2c_handle_t;
 
void i2c1_setup(i2c_handle_t* i2c, const uint32_t sda_pin, const uint32_t scl_pin);
void i2c2_setup(i2c_handle_t* i2c, const uint32_t sda_pin, const uint32_t scl_pin);
void i2c3_setup(i2c_handle_t* i2c, const uint32_t sda_port,const uint32_t sda_pin, const uint32_t scl_port, const uint32_t scl_pin);
void i2c_teardown(i2c_handle_t* i2c, bool teardown_gpio_clocks);
void i2c_enable_slow_mode(const i2c_handle_t* i2c);
void i2c_enable_fast_mode(const i2c_handle_t* i2c);
uint8_t i2c_read_register(const i2c_handle_t* i2c, const uint8_t device_address, const uint8_t device_register);
void i2c_read_registers_burst(const i2c_handle_t* i2c, uint8_t* data, const uint16_t bytes_to_read, const uint8_t device_address, const uint8_t device_register);
void i2c_write_register(const i2c_handle_t* i2c, uint8_t byte, const uint8_t device_address, const uint8_t device_register);
void i2c_write_registers_burst(const i2c_handle_t* i2c, const uint8_t* data, const uint16_t bytes_to_send, const uint8_t device_address, const uint8_t device_register);

#endif/* INC_I2C_H */
