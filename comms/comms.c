#include "core/comms.h"
#include "core/uart.h"
#include "core/crc.h"
#include <string.h>

#define PACKET_BUFFER_LENGTH    (8)

typedef enum comms_state_t  {
    CommsState_length,
    CommsState_payload,
    CommsState_crc,
} comms_state_t;

static uint8_t data_byte_count = 0;
static comms_state_t state = CommsState_length;

static comms_packet_t temporary_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t re_tx_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t ack_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t last_tx_packet = {
    .length = 0,
    .payload = {0},
    .crc = 0
};
static comms_packet_t packet_buffer[PACKET_BUFFER_LENGTH];
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFFER_LENGTH -1;


void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte)  {
    
    memset(packet, 0xFF, sizeof(comms_packet_t));
    packet->length = 1;
    packet->payload[0] = byte;
    packet->crc = comms_compute_crc(packet);
}

bool comms_is_single_byte_packet(const comms_packet_t* packet, uint8_t byte)  {
    if(packet->length!=1)   {
        return false;
    }
    if(packet->payload[0]!=byte)   {
        return false;
    }
    for (uint8_t i = 1; i < PACKET_PAYLOAD_LENGTH; i++) {
        if(packet->payload[i]!=0xff)    {
            return false;
        }
    }
    return true;
}


void comms_setup(void)  {

    // Initialize the retransmit and ack packet structure
    comms_create_single_byte_packet(&re_tx_packet, PACKET_RE_TX_PAYLOAD);
    comms_create_single_byte_packet(&ack_packet, PACKET_ACK_PAYLOAD);

}

void comms_update(void) {
    while (uart_data_available())   {
        switch (state)  {
            case CommsState_length:  {
                temporary_packet.length = uart_read_byte();
                state = CommsState_payload;
                break;
            }

            case CommsState_payload:    {
                temporary_packet.payload[data_byte_count++] = uart_read_byte();
                if(data_byte_count >= PACKET_PAYLOAD_LENGTH )   {
                    data_byte_count = 0;
                    state = CommsState_crc;
                }
                break;
            }

            case CommsState_crc:    {
                temporary_packet.crc = uart_read_byte();
                
                if(temporary_packet.crc != comms_compute_crc(&temporary_packet))    {
                    comms_write(&re_tx_packet);
                    state = CommsState_length;
                    break;
                }
                if(comms_is_single_byte_packet(&temporary_packet, PACKET_RE_TX_PAYLOAD))     {
                    comms_write(&last_tx_packet);
                    state = CommsState_length;
                    break;
                }
                if(comms_is_single_byte_packet(&temporary_packet, PACKET_ACK_PAYLOAD))  {
                    state = CommsState_length;
                    break;
                }
                
                uint32_t next_write_index = (packet_write_index+ 1 ) & packet_buffer_mask;
                if(next_write_index==packet_read_index) {
                    __asm__("BKPT #0");
                }

                memcpy(&packet_buffer[packet_write_index],&temporary_packet, sizeof(comms_packet_t));
                packet_write_index = next_write_index;
                comms_write(&ack_packet);
                state = CommsState_length;
                break;
            }
            default:    {
                state = CommsState_length;
                break;
            }
        }

    }

}

bool comms_packets_available(void)  {

    return packet_read_index != packet_write_index;
}

void comms_write(comms_packet_t* packet)    {
    uart_write((uint8_t*)packet, PACKET_LENGTH);
    memcpy(&last_tx_packet,packet, sizeof(comms_packet_t));

}

void comms_read(comms_packet_t* packet) {
    memcpy(packet, &packet_buffer[packet_read_index & packet_buffer_mask], sizeof(comms_packet_t));
    packet_read_index = (packet_read_index + 1) & packet_buffer_mask;

}

uint8_t comms_compute_crc(comms_packet_t* packet)   {
    return crc8((uint8_t*)packet, (PACKET_LENGTH - PACKET_CRC_BYTES));
}

