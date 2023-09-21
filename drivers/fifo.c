
#include "core/fifo.h"

void fifo_buffer_setup(fifo_buffer_t* fifo, uint8_t* buffer, uint32_t size) {

    fifo->buffer = buffer;
    fifo->read_index = 0;
    fifo->write_index = 0;
    fifo->mask = size - 1;

}

bool fifo_buffer_empty(fifo_buffer_t* fifo) {

    // Return true if read index and write index are the same
    return fifo->read_index == fifo->write_index;
}

bool fifo_buffer_write(fifo_buffer_t* fifo, uint8_t byte)   {
    
    // Copy the indices locally in case they change
    uint32_t local_read_index = fifo->read_index;
    uint32_t local_write_index = fifo->write_index;
    uint32_t next_write_index = (local_write_index + 1) & fifo->mask;

    // Make sure we aren't overrunning
    if(next_write_index == local_read_index)   {
        return false;
    }

    // Write data and increment index
    fifo->buffer[local_write_index] = byte;
    fifo->write_index = next_write_index;

    return true;

}

bool fifo_buffer_read(fifo_buffer_t* fifo, uint8_t* data) {

    // Copy the indices locally in case they change
    uint32_t local_read_index = fifo->read_index;
    uint32_t local_write_index = fifo->write_index;

    // return if theres no data to read
    if(local_read_index == local_write_index)   {
        return false;
    }

    // Copy data to the data pointer we got passed, increment the index
    *data = fifo->buffer[local_read_index];
    local_read_index = (local_read_index + 1) & fifo->mask;
    fifo->read_index = local_read_index;

    return true; 
}
