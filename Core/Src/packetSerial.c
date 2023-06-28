#include "packetSerial.h"

void decodePacket( uint8_t *data, uint16_t size ){
    uint16_t next0 = data[0];
    for( uint16_t i = 1; i < size; i++){
        if( i == next0 ){
            next0 += data[i];
            data[i] = 0;
        }
    }
}

void encodePacket( uint8_t *data, uint16_t size ){
    uint8_t *zerobuf = data;
    data[0] = data[size-1] = 0;
    for( uint16_t i = 1; i < size; i++){
        if( data[i] == 0 ){
            *zerobuf = &data[i] - zerobuf;
            zerobuf = &data[i];
        }
    }
}