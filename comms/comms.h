#ifndef INC_COMMS_H
#define INC_COMMS_H

#include "common-defines.h"

#define PACKET_PAYLOAD_LENGTH                   (16)
#define PACKET_LENGTH_BYTES                     (1)
#define PACKET_CRC_BYTES                        (1)
#define PACKET_LENGTH                           (PACKET_LENGTH_BYTES + PACKET_PAYLOAD_LENGTH + PACKET_CRC_BYTES)

#define PACKET_RE_TX_PAYLOAD                    (0x19)
#define PACKET_ACK_PAYLOAD                      (0x15)

#define BL_PACKET_SYNC_OBSERVED_PAYLOAD         (0x20)
#define BL_PACKET_FW_UPDATE_REQ_PAYLOAD         (0x31)
#define BL_PACKET_FW_UPDATE_RES_PAYLOAD         (0x37)
#define BL_PACKET_DEVICE_ID_REQ_PAYLOAD         (0x3C)
#define BL_PACKET_DEVICE_ID_RES_PAYLOAD         (0x3F)
#define BL_PACKET_FW_LENGTH_REQ_PAYLOAD         (0x42)
#define BL_PACKET_FW_LENGTH_RES_PAYLOAD         (0x45)
#define BL_PACKET_READY_FOR_FW_PAYLOAD          (0x48)
#define BL_PACKET_UPDATE_SUCCESSFUL_PAYLOAD     (0x54)
#define BL_PACKET_NACK                          (0x59)

#define SYNC_SEQ_0                              (0xC4)
#define SYNC_SEQ_1                              (0x55)
#define SYNC_SEQ_2                              (0x7E)
#define SYNC_SEQ_3                              (0x10)

typedef struct comms_packet_t   {
    uint8_t length;
    uint8_t payload[PACKET_PAYLOAD_LENGTH];
    uint8_t crc;
} comms_packet_t;

void comms_setup(void);
void comms_update(void);

bool comms_packets_available(void);
void comms_write(comms_packet_t* packet);
void comms_read(comms_packet_t* packet);
uint8_t comms_compute_crc(comms_packet_t* packet);
void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte);
bool comms_is_single_byte_packet(const comms_packet_t* packet, uint8_t byte);

#endif/* INC_COMMS_H */
