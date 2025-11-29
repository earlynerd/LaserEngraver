#ifndef LMCV4_PROTOCOL_H
#define LMCV4_PROTOCOL_H

#include <Arduino.h>

// USB Identification
#define LMC_VID 0x9588
#define LMC_PID 0x9899

// Command Opcodes
#define CMD_JUMP_ABS    0x8001
#define CMD_MARK_ABS    0x8002
#define CMD_GPIO_WRITE  0x8003
#define CMD_LSR_FREQ    0x8005
#define CMD_LSR_POWER   0x8006
#define CMD_LSR_DELAY   0x800D
#define CMD_LSR_CTRL    0x8018
#define CMD_SET_PARAM   0x801A
#define CMD_LIST_END    0x8000

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