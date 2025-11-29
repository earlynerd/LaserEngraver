#ifndef LMCV4_PROTOCOL_H
#define LMCV4_PROTOCOL_H

#include <Arduino.h>


// Status Flags
#define STATUS_BUSY     (1 << 0)
#define STATUS_ERROR    (1 << 1)
#define STATUS_BUFFER_LOW (1 << 2)

// Status Packet (12 bytes)
typedef struct __attribute__((packed)) {
    uint16_t statusFlags;
    uint16_t galvoX;
    uint16_t galvoY;
    uint16_t gpioIn;
    uint32_t reserved;
} LMC_Status_t;

#endif