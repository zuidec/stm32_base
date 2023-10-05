#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include "spi.h"



static uint8_t data_buffer[FIFO_BUFFER_SIZE] = {0U};

fifo_buffer_t* spi1_isr_fifo_address;

void spi1_isr(void) {
    // Reset interrupt flags


    // Catch the data then write it to the fifo buffer
    // Not sure if this will work passing the address of the fifo as a uint32_t
    if(!fifo_buffer_write(spi1_isr_fifo_address, (uint8_t)spi_read(SPI1)))  {
        // Handle write failure
    }
    
}

void spi_setup(spi_handle_t* spi, const uint32_t spi_address)    {

    switch(spi_address) { 
        case SPI1:  {
            spi->spi_base_address = spi_address;
            spi->rcc_address = RCC_SPI1;
            spi->rcc_reset_address = RST_SPI1;
            spi->nvic_irq_id = NVIC_SPI1_IRQ;
            spi1_isr_fifo_address = &spi->fifo;
            
            break;
        }
        case SPI2:  {
            spi->spi_base_address = spi_address;
            spi->rcc_address = RCC_SPI2;
            spi->rcc_reset_address = RST_SPI2;
            spi->nvic_irq_id = NVIC_SPI2_IRQ;
            
            break;
        }
        case SPI3:  {
            spi->spi_base_address = spi_address;
            spi->rcc_address = RCC_SPI3;
            spi->rcc_reset_address = RST_SPI3;
            spi->nvic_irq_id = NVIC_SPI3_IRQ;
            
            break;
        }
        default:    {
            return; // Return if no valid SPI address
            break;
        }
    }

    rcc_periph_clock_enable(spi->rcc_address);
    rcc_periph_reset_pulse(spi->rcc_reset_address);

    fifo_buffer_setup(&spi->fifo, data_buffer, FIFO_BUFFER_SIZE);

    spi_init_master(spi->spi_base_address, 
                    SPI_CR1_BAUDRATE_FPCLK_DIV_128, // 84MHz / 128 = 656 kHz; 1 Mhz is max for MPU6050 
                    SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, 
                    SPI_CR1_CPHA_CLK_TRANSITION_2,
                    SPI_CR1_DFF_8BIT, 
                    SPI_CR1_MSBFIRST );

    spi_enable_software_slave_management(spi->spi_base_address);
	spi_set_nss_high(spi->spi_base_address);

    spi_enable_rx_buffer_not_empty_interrupt(spi->spi_base_address);
    nvic_enable_irq(spi->nvic_irq_id);

    spi_enable(spi->spi_base_address);

}

void spi_teardown(spi_handle_t* spi) {
    

    
    spi_disable_rx_buffer_not_empty_interrupt(SPI1);

    nvic_disable_irq(spi->nvic_irq_id);
    spi_clean_disable(spi->spi_base_address);
    spi_set_nss_low(spi->spi_base_address);
    spi_disable_software_slave_management(spi->spi_base_address);

    rcc_periph_clock_disable(spi->rcc_address);
}

uint32_t spi_read_bytes(spi_handle_t* spi, uint8_t* data, uint32_t bytes_to_read)   {
    
    if(bytes_to_read == 0) {
        return 0;
    }

    for(uint32_t bytes_read = 0; bytes_read < bytes_to_read; bytes_read++) {
        if(!fifo_buffer_read(&spi->fifo, &data[bytes_read])) {
            return bytes_read;
        }
    }

    return bytes_to_read;
}

uint8_t spi_read_byte(spi_handle_t* spi) {

    uint8_t byte = 0;
    (void)spi_read_bytes(spi, &byte,1);
    return byte;
}

void spi_write_bytes(spi_handle_t* spi, uint8_t* data, uint32_t length)  {

    for(uint32_t i =0; i< length; i++)  {
        spi_write_byte(spi, data[i]);
    }
}

void spi_write_byte(spi_handle_t* spi, uint8_t byte)   {

    spi_send(spi->spi_base_address, (uint16_t)byte);
}

bool spi_data_available(spi_handle_t* spi)   {

    return !fifo_buffer_empty(&spi->fifo);
}

