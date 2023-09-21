#ifndef INC_FIFO_H
#define INC_FIFO_H

#include "common-defines.h"

typedef struct fifo_buffer_t    {
    
    uint8_t* buffer;
    uint32_t mask;
    uint32_t read_index;
    uint32_t write_index;
} fifo_buffer_t;

void fifo_buffer_setup(fifo_buffer_t* fifo, uint8_t* buffer, uint32_t size);
bool fifo_buffer_empty(fifo_buffer_t* fifo);
bool fifo_buffer_write(fifo_buffer_t* fifo, uint8_t byte);
bool fifo_buffer_read(fifo_buffer_t* fifo, uint8_t* data);

#endif/* INC_FIFO_H */
