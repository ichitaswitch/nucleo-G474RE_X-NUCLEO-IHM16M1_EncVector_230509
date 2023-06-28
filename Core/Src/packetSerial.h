#ifndef PACKET_SERIAL_H
#define PACKET_SERIAL_H

#include <stdint.h>

void decodePacket( uint8_t *data, uint16_t size );
void encodePacket( uint8_t *data, uint16_t size );

#endif