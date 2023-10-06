#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <string.h>
#include "core/system.h"
#include "drivers/uart/uart.h"
#include "drivers/i2c/i2c.h"
/*#include "../core/system.h"
#include "../drivers/uart/uart.h"
#include "../drivers/i2c/i2c.h"*/

#define LED_PORT            (GPIOC)
#define LED_PIN             (GPIO13)

#define I2C1_PORT           (GPIOB)
#define I2C1_SDA            (GPIO7)
#define I2C1_SCL            (GPIO6)

#define UART2_PORT          (GPIOA)
#define RX2_PIN             (GPIO3)
#define TX2_PIN             (GPIO2)
i2c_handle_t i2c1;
uint8_t buffer[128] = {0};
fifo_buffer_t fifo;
static void gpio_setup(void)    {
    
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    
    // Set gpio to alternate function, then set the alternate function to AF1 (TIM2)
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_clear(LED_PORT, LED_PIN);
    // Set up UART2 and change the pins to alternate function
    gpio_mode_setup(UART2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TX2_PIN | RX2_PIN );
    gpio_set_af(UART2_PORT,GPIO_AF7, TX2_PIN | RX2_PIN );
    
}

int main(void)  {
    gpio_setup();
    system_setup();
    uart_setup();
    fifo_buffer_setup(&fifo, buffer, 128);
    i2c_setup(&i2c1, I2C1_BASE, I2C1_PORT, I2C1_SDA, I2C1_SCL, &fifo);
    uint8_t data = 0;
    uart_write_byte(122);
    gpio_toggle(LED_PORT, LED_PIN);
    uint64_t millis = system_get_ticks();
    
    while(1)    {
      //  
        if(system_get_ticks()-millis >=2000) {
            millis = system_get_ticks();
            gpio_toggle(LED_PORT, LED_PIN);
            uart_write_byte(0x00);
            uart_write_byte(0x3A);
            uart_write_byte(0x20);
            uart_write_byte(0x20);

            for(uint16_t i=0x00; i<=0xFF; i++)   {
                data = i2c_read_byte(&i2c1,0x68, i);
                uart_write_byte(data);
                uart_write_byte(0x20);
                if(i%16==0&&i!=0xFF) {
                    uart_write_byte(0x0A);
                    uart_write_byte(i);
                    uart_write_byte(0x3A);
                    uart_write_byte(0x20);
                    uart_write_byte(0x20);
                }
                data = 0x00;
            }
        }
    }
}
