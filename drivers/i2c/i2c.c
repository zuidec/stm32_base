#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "i2c.h"

static void i2c_master_start(const i2c_handle_t* i2c, const uint8_t rw_bit);
static void i2c_master_stop(const i2c_handle_t* i2c);
static void i2c_master_send_address(const i2c_handle_t* i2c, const uint8_t device_address, const uint8_t rw_bit);
static void i2c_master_send_byte(const i2c_handle_t* i2c, const uint8_t data);
static void i2c_master_send_burst(const i2c_handle_t* i2c, const uint8_t* data, const uint16_t bytes_to_send);
static uint8_t i2c_master_read_byte(const i2c_handle_t* i2c);
static void i2c_master_read_burst(const i2c_handle_t* i2c, uint8_t* data, const uint16_t bytes_to_read);


void i2c1_setup(i2c_handle_t* i2c, const uint32_t sda_pin, const uint32_t scl_pin)   {
    
    // Pass values to i2c_handle to initialize
    i2c->i2c_base_address   = I2C1_BASE;
    i2c->gpio_port          = GPIOB;
    i2c->sda_pin            = sda_pin;
    i2c->scl_pin            = scl_pin;
    
    // Ensure peripheral is disabled before configuration
    i2c_peripheral_disable(i2c->i2c_base_address);
    
    // Enable GPIO and set alternate function for i2c: all I2C1 pins are on port B
    rcc_periph_clock_enable(RCC_GPIOB);
    
    // Set pins to pullup and use alternate function AF4: all I2C1 pins use AF4
    gpio_mode_setup(i2c->gpio_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, sda_pin | scl_pin);
    gpio_set_af(i2c->gpio_port, GPIO_AF4, sda_pin | scl_pin);
    
    // Set pins to use max possible speed and output drain
    gpio_set_output_options(i2c->gpio_port,GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, sda_pin|scl_pin);
    
    // Start peripheral clock for i2c
    rcc_periph_clock_enable(RCC_I2C1);
    
    // Reset i2c
    I2C1_CR1 |= I2C_CR1_SWRST;
    I2C1_CR1 &= ~I2C_CR1_SWRST;
    
    // Set CR2 to APB1 freq as a whole number in MHz, e.g. 42MHz = 42
    uint32_t i2c_freq = rcc_get_i2c_clk_freq(i2c->i2c_base_address);
    I2C1_CR2 =  i2c_freq / HZ_TO_MHZ;
    
    // Set CCR register, see reference manual for calculation. Start in SM by default
    I2C1_CCR = I2C_SM_T_LOW / (1/ i2c_freq); //I2C1_CCR = (210); 
    
    // Configure trise in SM by default, see reference manual for calculation
    I2C1_TRISE = (I2C_SM_MAX_SCL_RISE   /  ( (1/i2c_freq) * S_TO_NS) ) + 1;
    
    // Enable i2c
    i2c_peripheral_enable(i2c->i2c_base_address);
   
}

void i2c2_setup(i2c_handle_t* i2c, const uint32_t sda_pin, const uint32_t scl_pin){

    // Pass values to i2c_handle to initialize
    i2c->i2c_base_address   = I2C2_BASE;
    i2c->gpio_port          = GPIOB;
    i2c->sda_pin            = sda_pin;
    i2c->scl_pin            = scl_pin;
    
    // Ensure peripheral is disabled before configuration
    i2c_peripheral_disable(i2c->i2c_base_address);
    
    // Enable GPIO and set alternate function for i2c: all I2C2 pins are on port B
    rcc_periph_clock_enable(RCC_GPIOB);
    
    // Set pins to pullup and use alternate function AF4: all I2C1 pins use AF4
    gpio_mode_setup(i2c->gpio_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, sda_pin | scl_pin);
    gpio_set_af(i2c->gpio_port, GPIO_AF4, sda_pin | scl_pin);
    
    // Set pins to use max possible speed and output drain
    gpio_set_output_options(i2c->gpio_port,GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, sda_pin|scl_pin);
    
    // Start peripheral clock for i2c
    rcc_periph_clock_enable(RCC_I2C2);
    
    // Reset i2c
    I2C2_CR1 |= I2C_CR1_SWRST;
    I2C2_CR1 &= ~I2C_CR1_SWRST;
    
    // Set CR2 to APB1 freq as a whole number in MHz, e.g. 42MHz = 42
    uint32_t i2c_freq = rcc_get_i2c_clk_freq(i2c->i2c_base_address);
    I2C2_CR2 =  i2c_freq / HZ_TO_MHZ;
    
    // Set CCR register, see reference manual for calculation. Start in SM by default
    I2C2_CCR = I2C_SM_T_LOW / (1/ i2c_freq); //I2C1_CCR = (210); 
    
    // Configure trise in SM by default, see reference manual for calculation
    I2C2_TRISE = (I2C_SM_MAX_SCL_RISE   /  ( (1/i2c_freq) * S_TO_NS) ) + 1;
    
    // Enable i2c
    i2c_peripheral_enable(i2c->i2c_base_address);
 
}

void i2c3_setup(i2c_handle_t* i2c, const uint32_t sda_port, const uint32_t sda_pin, const uint32_t scl_port, const uint32_t scl_pin)    {
    
    // Pass values to i2c_handle to initialize
    i2c->i2c_base_address   = I2C3_BASE;
    i2c->gpio_port          = sda_port;
    i2c->sda_pin            = sda_pin;
    i2c->scl_pin            = scl_pin;
    
    // Ensure peripheral is disabled before configuration
    i2c_peripheral_disable(i2c->i2c_base_address);
    
    // I2C3 has pins spread across ports A, B, and C so setup is a little more complicated
    // Enable clock for gpio port for the SDA
    if(sda_port == GPIOB)   {
        rcc_periph_clock_enable(RCC_GPIOB);
    }
    else if(sda_port == GPIOC)   {
        rcc_periph_clock_enable(RCC_GPIOC);
    }
    // Enable GPIO and set alternate function for i2c: I2C3 SCL is only on port A
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Set pins to pullup and use alternate function AF4: all I2C1 pins use AF4
    gpio_mode_setup(sda_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, sda_pin);
    gpio_mode_setup(scl_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, scl_pin);
    
    // Set AF for SDA based on which port it's on
    if(sda_port == GPIOC)   {
        gpio_set_af(sda_port, GPIO_AF4, sda_pin);
    }
    else{
        gpio_set_af(sda_port, GPIO_AF9, sda_pin);
    }
    // SCL only available on AF4
    gpio_set_af(scl_port, GPIO_AF4, scl_pin);
    
    // Set pins to use max possible speed and output drain
    gpio_set_output_options(sda_port,GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, sda_pin);
    gpio_set_output_options(scl_port,GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, scl_pin);
    
    // Start peripheral clock for i2c
    rcc_periph_clock_enable(RCC_I2C3);
    
    // Reset i2c
    I2C3_CR1 |= I2C_CR1_SWRST;
    I2C3_CR1 &= ~I2C_CR1_SWRST;
    
    // Set CR2 to APB1 freq as a whole number in MHz, e.g. 42MHz = 42
    uint32_t i2c_freq = rcc_get_i2c_clk_freq(i2c->i2c_base_address);
    I2C3_CR2 =  i2c_freq / HZ_TO_MHZ;
    
    // Set CCR register, see reference manual for calculation. Start in SM by default
    I2C3_CCR = I2C_SM_T_LOW / (1/ i2c_freq); //I2C1_CCR = (210); 
    
    // Configure trise in SM by default, see reference manual for calculation
    I2C3_TRISE = (I2C_SM_MAX_SCL_RISE   /  ( (1/i2c_freq) * S_TO_NS) ) + 1;
    
    // Enable i2c
    i2c_peripheral_enable(i2c->i2c_base_address);
 
}

void i2c_teardown(i2c_handle_t* i2c, bool teardown_gpio_clocks)    {
    
    /*  
     *  Disable the peripheral and RCC clocks
     *  We are leaving the CR and CCR registers configured, 
     *  they will be overwritten in the next setup
     */
    
    i2c_peripheral_disable(i2c->i2c_base_address);
    
    if(i2c->i2c_base_address == I2C1)   {
        rcc_periph_clock_disable(RCC_I2C1);
        // Teardown gpio clocks if indicated
        if(teardown_gpio_clocks)    {
            rcc_periph_clock_disable(i2c->gpio_port);
        }
    }
    else if(i2c->i2c_base_address == I2C2)   {
        rcc_periph_clock_disable(RCC_I2C2);
        // Teardown gpio clocks if indicated
        if(teardown_gpio_clocks)    {
            rcc_periph_clock_disable(i2c->gpio_port);
        }
    }
    else if(i2c->i2c_base_address == I2C3)   {
        rcc_periph_clock_disable(RCC_I2C3);
        // Teardown gpio clocks if indicated
        if(teardown_gpio_clocks)    {
            // Taking down this will take down the SDA port
            rcc_periph_clock_disable(i2c->gpio_port);
            // Taking down GPIOA as well, since I2C3 SCL port is here
            rcc_periph_clock_disable(GPIOA);
        }
    }
    
}

void i2c_read_registers_burst(const i2c_handle_t* i2c, uint8_t* data, const uint16_t bytes_to_read, const uint8_t device_address, const uint8_t device_register)   {
    
    //  Burst read sequence
    //  M:  S AD+w      RA      S AD+r           ACK      ACK...      NACK ST
    //  S:          ACK     ACK         ACK DATA     Data    ... Data  
    
    // Generate start
    i2c_master_start(i2c, I2C_WRITE);
    
    // AD+w
    i2c_master_send_address(i2c, device_address, I2C_WRITE);
    
    // Send Register Address
    i2c_master_send_byte(i2c, device_register);
    
    // Generate restart
    i2c_master_stop(i2c);
    i2c_master_start(i2c, I2C_READ);
    
    // Send AD+r
    i2c_master_send_address(i2c, device_address, I2C_READ);
    
    // Read the data byte
    i2c_master_read_burst(i2c, data, bytes_to_read);

    // Generate stop
    i2c_master_stop(i2c);    

}

uint8_t i2c_read_register(const i2c_handle_t* i2c, const uint8_t device_address, const uint8_t device_register)    {

    //  Single byte sequence
    //  M:  S AD+w      RA      S AD+r          NACK ST
    //  S:          ACK     ACK         ACK DATA
    
    // Generate start
    i2c_master_start(i2c, I2C_WRITE);
    
    // AD+w
    i2c_master_send_address(i2c, device_address, I2C_WRITE);
    
    // Send Register Address
    i2c_master_send_byte(i2c, device_register);
    
    // Generate restart
    i2c_master_stop(i2c);
    i2c_master_start(i2c, I2C_READ);
    
    // Send AD+r
    i2c_master_send_address(i2c, device_address, I2C_READ);
    
    // Read the data byte
    uint8_t data = i2c_master_read_byte(i2c);

    // Generate stop
    i2c_master_stop(i2c);    

    return data;
}

void i2c_write_registers_burst(const i2c_handle_t* i2c, const uint8_t* data, const uint16_t bytes_to_send, const uint8_t device_address, const uint8_t device_register) {
    
    //  Write sequence
    //  M:  S AD+w      RAdd        Data        Data    ... Data        STOP
    //  S:         ACK         ACK        ACK       ACK ...       ACK

    // Generate start
    i2c_master_start(i2c,I2C_WRITE);

    // Send address
    i2c_master_send_address(i2c, device_address, I2C_WRITE);

    // Send register address
    i2c_master_send_byte(i2c,device_register);

    // Send data to be written
    i2c_master_send_burst(i2c, data, bytes_to_send);

    // Generate stop
    i2c_master_stop(i2c);

}

void i2c_write_register(const i2c_handle_t* i2c, const uint8_t byte, const uint8_t device_address, const uint8_t device_register)    {
    
    //  Write sequence
    //  M:  S AD+w      RAdd        Data       STOP
    //  S:         ACK         ACK        ACK  

    // Generate start
    i2c_master_start(i2c,I2C_WRITE);

    // Send address
    i2c_master_send_address(i2c, device_address, I2C_WRITE);

    // Send register address
    i2c_master_send_byte(i2c,device_register);

    // Send data to be written
    i2c_master_send_byte(i2c,byte);

    // Generate stop
    i2c_master_stop(i2c);

}

void i2c_enable_slow_mode(const i2c_handle_t* i2c) {
    
    // I2C must be disabled to change the mode
    i2c_peripheral_disable(i2c->i2c_base_address);

    // Set CR2 to APB1 freq as a whole number in MHz, e.g. 42MHz = 42
    uint32_t i2c_freq = rcc_get_i2c_clk_freq(i2c->i2c_base_address);
    I2C_CR2(i2c->i2c_base_address) =  i2c_freq / HZ_TO_MHZ;
    
    // Set CCR register, see reference manual for calculation
    I2C_CCR(i2c->i2c_base_address) &= ~(I2C_CCR_FS); 
    I2C_CCR(i2c->i2c_base_address) |= (I2C_SM_T_LOW / (1 / i2c_freq)); 
    
    // Configure trise , see reference manual for calculation
    I2C_TRISE(i2c->i2c_base_address) = (I2C_SM_MAX_SCL_RISE   /  ( (1 / i2c_freq) * S_TO_NS) ) + 1;
    
    // Re-enable i2c
    i2c_peripheral_enable(i2c->i2c_base_address);

}

void i2c_enable_fast_mode(const i2c_handle_t* i2c) {
    
    // I2C must be disabled to change the mode
    i2c_peripheral_disable(i2c->i2c_base_address);

    // Set CR2 to APB1 freq as a whole number in MHz, e.g. 42MHz = 42
    uint32_t i2c_freq = rcc_get_i2c_clk_freq(i2c->i2c_base_address);
    I2C_CR2(i2c->i2c_base_address) =  i2c_freq / HZ_TO_MHZ;
    
    // Set CCR register, see reference manual for calculation
    I2C_CCR(i2c->i2c_base_address) = (I2C_CCR_FS) | ( (I2C_FM_T_LOW / (1 / i2c_freq) ) * 2); 
    
    // Configure trise , see reference manual for calculation
    I2C_TRISE(i2c->i2c_base_address) = (I2C_FM_MAX_SCL_RISE   /  ( (1 / i2c_freq) * S_TO_NS) ) + 1;
    
    // Re-enable i2c
    i2c_peripheral_enable(i2c->i2c_base_address);

}

/*
 *  
 *  Private static functions
 * 
 */

static void i2c_master_start(const i2c_handle_t* i2c, const uint8_t rw_bit) {
    
    // Generate start
    I2C_CR1(i2c->i2c_base_address) &= ~I2C_CR1_POS;
    if(rw_bit)  {
        // Enable ACK if reading
        I2C_CR1(i2c->i2c_base_address) |= I2C_CR1_ACK;
    }
    I2C_CR1(i2c->i2c_base_address) |= (I2C_CR1_START);
    // Wait for start bit to set (EV5)
    while(!(I2C_SR1(i2c->i2c_base_address) & I2C_SR1_SB))   {}

}

static void i2c_master_send_address(const i2c_handle_t* i2c, const uint8_t device_address, const uint8_t rw_bit)  {
    
    volatile uint16_t reset = 0x00;
    // Place address with correct r/w bit into DR 
    I2C_DR(i2c->i2c_base_address) = (device_address << 1 ) | rw_bit;
    // Wait for ADDR flag to set (EV6)
    while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_ADDR))   {}
    // Read SR1 and SR2 to clear flag
    reset = I2C_SR1(i2c->i2c_base_address);
    reset = I2C_SR2(i2c->i2c_base_address);

}

static void i2c_master_send_byte(const i2c_handle_t* i2c, const uint8_t data)   {
    
    // Wait for TX buffer to empty then place data into DR (EV 8_1)
    while(!(  I2C_SR1(i2c->i2c_base_address) & I2C_SR1_TxE))   {}
    I2C_DR(i2c->i2c_base_address) = data;
    // Wait for TX buffer to empty again and for BTF flag to set (EV 8_2)
    while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_TxE))   {}
    while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_BTF))   {}
    // Clear flags by reading SR1 and SR2
    volatile uint16_t reg =  I2C_SR1(i2c->i2c_base_address);
    reg = I2C_SR2(i2c->i2c_base_address);

}

static void i2c_master_send_burst(const i2c_handle_t* i2c, const uint8_t* data, const uint16_t bytes_to_send)   {
    
    volatile uint16_t reg = 0;
    // Wait for Tx buffer to empty then place data into DR (EV 8_1)
    while(!(  I2C_SR1(i2c->i2c_base_address) & I2C_SR1_TxE))   {}
    
    // Loop through data to be sent one byte at a time
    for(uint16_t i = 0; i < bytes_to_send; i++) {
        // Place data into DR
        I2C_DR(i2c->i2c_base_address) = data[i];
        // Wait for TX to empty and BTF to set (EV 8_2)
        while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_TxE))   {}
        while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_BTF))   {}
        // Clear flags by reading SR1 and SR2
        reg = I2C_SR1(i2c->i2c_base_address);
        reg = I2C_SR2(i2c->i2c_base_address);
    }
}

static uint8_t i2c_master_read_byte(const i2c_handle_t* i2c)   {
    
    // Wait for RX buffer to have data (EV 7)
    while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_RxNE))   {}
    // ACK data receipt - for single byte this must happen BEFORE reading DR
    I2C_CR1(i2c->i2c_base_address) |= I2C_CR1_ACK;
    // Read byte from DR
    uint8_t data = I2C_DR(i2c->i2c_base_address);
    // Clear flags by reading SR1 and SR2
    volatile uint16_t reg = I2C_SR1(i2c->i2c_base_address);
    reg = I2C_SR2(i2c->i2c_base_address);

    return data;
}

static void i2c_master_read_burst(const i2c_handle_t* i2c, uint8_t* data, const uint16_t bytes_to_read)   {
    
    // Loop through reading one byte at a time until complete
    for(uint16_t i = 0; i < (bytes_to_read -1); i++) {
        // Wait for RX buffer to have data (EV 7)
        while(!( I2C_SR1(i2c->i2c_base_address) & I2C_SR1_RxNE))   {}
        // Read data from DR 
        data[i] = I2C_DR(i2c->i2c_base_address);
        // ACK receipt - ACK AFTER reading DR until the last byte
        I2C_CR1(i2c->i2c_base_address) |= I2C_CR1_ACK;
    }
    // Read the last byte
    data[(bytes_to_read-1)] = I2C_DR(i2c->i2c_base_address);
    // Clear flags by reading SR1 and SR2
    volatile uint16_t reg = I2C_SR1(i2c->i2c_base_address);
    reg = I2C_SR2(i2c->i2c_base_address);

}

static void i2c_master_stop(const i2c_handle_t* i2c) {
    
    // Turn off ACK - only used to receive, but safe to clear the bit regardless
    I2C_CR1(i2c->i2c_base_address) &= ~I2C_CR1_ACK;
    // Generate stop condition
    I2C_CR1(i2c->i2c_base_address) |= I2C_CR1_STOP;
    // Clear flags by reading SR1 and SR2
    volatile uint16_t reg = I2C_SR1(i2c->i2c_base_address);
    reg = I2C_SR2(i2c->i2c_base_address);

}
