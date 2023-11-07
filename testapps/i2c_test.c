#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "core/system.h"
#include "drivers/uart/uart.h"
#include "drivers/i2c/i2c.h"
#include "devices/mpu60X0/mpu60X0.h"

#define LED_PORT            (GPIOC)
#define LED_PIN             (GPIO13)

#define I2C1_PORT           (GPIOB)
#define I2C1_SDA            (GPIO7)
#define I2C1_SCL            (GPIO6)

#define UART2_PORT          (GPIOA)
#define RX2_PIN             (GPIO3)
#define TX2_PIN             (GPIO2)

#define MPU_ADDR            (0x68)
#define BAUDRATE            (115200)

i2c_handle_t i2c1   = {0};
uart_t uart2        = {0};

uint8_t buffer[128] = {'\0'};
uint8_t output[128] = {'\0'};

fifo_buffer_t i2c_fifo;
fifo_buffer_t uart_fifo;
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
static void buffer_reset(uint8_t* data, uint16_t size)  {
    for(uint16_t i =0; i< size; i++)    {
        data[i] = '\0';
    }
}
int main(void)  {
    gpio_setup();
    system_setup();
    
    fifo_buffer_setup(&i2c_fifo, buffer, 128);
    fifo_buffer_setup(&uart_fifo, buffer, 128);
    
    i2c1_setup(&i2c1, (uint32_t)I2C1_SDA, (uint32_t)I2C1_SCL);

    uart2.uart_base = USART2;
    uart2.baudrate = BAUDRATE;
    uart2.fifo = &uart_fifo;
    uart_setup(&uart2);

    volatile uint8_t data = 1;

    uart_write(&uart2, (uint8_t*)"\nBoot complete\n",15);
    uart_write(&uart2, (uint8_t*)"Resetting MPU\n",14);
    system_delay(100);

    i2c_write_register(&i2c1, (1<<7), MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
    i2c_write_register(&i2c1, 0x07, MPU_ADDR, MPU60X0_REG_SIGNAL_PATH_RESET);
    system_delay(100);
    i2c_write_register(&i2c1, 0x07, MPU_ADDR, MPU60X0_REG_USER_CTRL);
    system_delay(100);
    i2c_write_register(&i2c1, 0x00, MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
    i2c_write_register(&i2c1, 0x02, MPU_ADDR, MPU60X0_REG_CONFIG);
    i2c_write_register(&i2c1, 0x08, MPU_ADDR, MPU60X0_REG_SMPLRT_DIV);
    
    uart_println(&uart2, (uint8_t*)"Verifying settings");
    //uart_write("Verifying settings\n",19);
    
    volatile uint8_t reg = 0;
    reg = i2c_read_register(&i2c1,MPU_ADDR,MPU60X0_REG_PWR_MGMT_1);    
    if(reg != 0x00)    {
        uart_println(&uart2, (uint8_t*)"PWRhMGMT_1 FAIL");
        while(data){data++;}
    }

    reg = i2c_read_register(&i2c1,MPU_ADDR,MPU60X0_REG_SIGNAL_PATH_RESET);    
    if(reg !=0x00)    {
        uart_println(&uart2, (uint8_t*)"SIGNAL_PATH_RESET FAIL");
        while(data){data++;}
    }

    reg = i2c_read_register(&i2c1,MPU_ADDR,MPU60X0_REG_USER_CTRL);    
    if(reg !=0x00)    {
        uart_println(&uart2, (uint8_t*)"USER_CTRL FAIL");
        while(data){data++;}
    }

    reg = i2c_read_register(&i2c1,MPU_ADDR,MPU60X0_REG_CONFIG);    
    if(reg !=0x02)    {
        uart_println(&uart2, (uint8_t*)"CONFIG FAIL");
        while(data){data++;}
    }

    reg = i2c_read_register(&i2c1,MPU_ADDR,MPU60X0_REG_SMPLRT_DIV);    
    if(reg !=0x08)    {
        uart_println(&uart2, (uint8_t*)"SMPLRT_DIV FAIL");
        while(data){data++;}
    }

    uart_println(&uart2, (uint8_t*)"Settings verified");
    gpio_toggle(LED_PORT, LED_PIN);
    uint64_t millis = system_get_ticks();
    volatile int16_t value = 0;
    volatile uint16_t buf_size = 0;
    while(1)    {
      //  
        if(system_get_ticks()-millis >=100) {
            millis = system_get_ticks();
            gpio_toggle(LED_PORT, LED_PIN);
            
            
            i2c_read_registers_burst(&i2c1,&buffer,6,MPU_ADDR, MPU60X0_REG_ACCEL_XOUT_H);
            value = (int16_t)((buffer[0] << 8) | buffer[1]);
            value = value*16384;
            buf_size = sprintf(output, "Accel x: %8i",value);
            uart_write(&uart2, output, buf_size);
            buffer_reset(&output[0], 128);//memset(&output[0], 0, 128);
            
            value = (int16_t)((buffer[2] << 8) | buffer[3]);
            value = value*16384;
            buf_size = sprintf(output, " Accel y: %8i",value);
            uart_write(&uart2, output, buf_size);
            buffer_reset(&output[0], 128);//memset(&output[0], 0, 128);
            
            value = (int16_t)((buffer[4] << 8) | buffer[5]);
            value = value*16384;
            buf_size = sprintf(output, " Accel z: %8i",value);
            uart_write(&uart2, output, buf_size);
            buffer_reset(&output[0], 128);//memset(&output[0], 0, 128);
            buffer_reset(&buffer[0], 128);//memset(&buffer[0], 0, 128);

            i2c_read_registers_burst(&i2c1,&buffer,6,MPU_ADDR, MPU60X0_REG_ACCEL_XOUT_H);
            value = (int16_t)((buffer[0] << 8) | buffer[1]);
            value = value/131;
            buf_size = sprintf(output, " Gyro x: %8i",value);
            uart_write(&uart2, output, buf_size);
            buffer_reset(&output[0], 128);//memset(&output[0], 0, 128);
            
            value = (int16_t)((buffer[2] << 8) | buffer[3]);
            value = value/131;
            buf_size = sprintf(output, " Gyro y: %8i",value);
            uart_write(&uart2, output, buf_size);
            buffer_reset(&output[0], 128);//memset(&output[0], 0, 128);
            
            value = (int16_t)((buffer[4] << 8) | buffer[5]);
            value = value/131;
            buf_size = sprintf(output, " Gyro z: %8i \n",value);
            uart_write(&uart2, output, buf_size);
            buffer_reset(&output[0], 128);//memset(&output[0], 0, 128);
            buffer_reset(&buffer[0], 128);
            //uart_write_16(&uart2, value);//memset(&buffer[0], 0, 128);
            
            /*
            uart_write(" gyro y: ",9);
            uart_write_16(value);
            uart_write(" gyro z: ",9);
            uart_write_16(value);
            uart_write_byte('\n');
            */
           // memset(&buffer, 0, 128);
            
            /*data = i2c_read_byte(&i2c1, MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
            uart_write_byte(data);
            uart_write_byte(0x20);
            data = i2c_read_byte(&i2c1, MPU_ADDR, MPU60X0_REG_WHO_AM_I);
            uart_write_byte(data);
            uart_write_byte(0x20);
            i2c_write_byte(&i2c1, (0<<7), MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
            //i2c_write_byte(&i2c1, (1<<7), MPU_ADDR, MPU60X0_REG_PWR_MGMT_2);
            data = i2c_read_byte(&i2c1, MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
            uart_write_byte(data);
            uart_write_byte(0x20);
            i2c_write_byte(&i2c1, (1<<7), MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
            //i2c_write_byte(&i2c1, (1<<7), MPU_ADDR, MPU60X0_REG_PWR_MGMT_2);
            data = i2c_read_byte(&i2c1, MPU_ADDR, MPU60X0_REG_PWR_MGMT_1);
            uart_write_byte(data);
            uart_write_byte(0x0A);
            uart_write_byte(0x00);
            uart_write_byte(0x3A);
            uart_write_byte(0x20);
            uart_write_byte(0x20);

            for(uint16_t i=0x00; i<=0xFF; i++)   {
                data = i2c_read_byte(&i2c1,MPU_ADDR, i);
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
            }*/
        }
    }
}
