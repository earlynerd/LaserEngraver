#ifndef LMCV4_PROTOCOL_H
#define LMCV4_PROTOCOL_H

#include <Arduino.h>

// Constants
#define LMCV4_VID           0x9588
#define LMCV4_PID           0x9899
#define CMD_SIZE            12
#define REPORT_SIZE         8

// Status Bitmasks (Byte 6 of Status Report)
#define LMC_STATUS_BUSY     (1 << 0) // 0x01
#define LMC_STATUS_ERROR    (1 << 1) // 0x02
#define LMC_STATUS_RUNNING  (1 << 2) // 0x04
#define LMC_STATUS_UNK_08   (1 << 3) // 0x08
#define LMC_STATUS_UNK_10   (1 << 4) // 0x10
#define LMC_STATUS_READY    (1 << 5) // 0x20 - Critical: Host waits for this bit
#define LMC_STATUS_UNK_40   (1 << 6) // 0x40
#define LMC_STATUS_UNK_80   (1 << 7) // 0x80

// Command Structure (Packed to match wire protocol)
struct BalorCommand {
    uint16_t opcode;
    uint16_t params[5];
} __attribute__((packed));

#endif