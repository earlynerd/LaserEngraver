#ifndef LASER_INTERFACE_H
#define LASER_INTERFACE_H

#include <Adafruit_TinyUSB.h>
#include "LMCV4_Protocol.h"
#include "RingBuffer.h"

// Buffer size: 4096 words (8KB). Must be power of 2.
#define CMD_BUFFER_SIZE 4096 

class LaserInterface : public Adafruit_USBD_Interface {
public:
    LaserInterface();
    
    // Initialize USB endpoints and descriptors
    bool begin();
    
    // Helper to pipe debug info to a hardware serial port
    void setDebugStream(Stream& stream);
    
    // Override the base class method to provide our custom descriptor
    uint16_t getInterfaceDescriptor(uint8_t none, uint8_t* buf, uint16_t bufsize) override;

    // Main loop task
    bool task();

    // Setters for hardware status to report back to host
    void updateStatus(uint16_t x, uint16_t y, uint16_t gpio);

    // Callbacks
    void (*onJumpTo)(uint16_t x, uint16_t y) = nullptr;
    void (*onMarkTo)(uint16_t x, uint16_t y) = nullptr;
    void (*onLaserControl)(uint16_t command) = nullptr;
    void (*onParamUpdate)(uint16_t paramId, uint16_t value) = nullptr;

private:
    // Endpoint Addresses (Allocated dynamically)
    uint8_t _ep_in;
    uint8_t _ep_out;
    uint8_t _itf;
    // Debugging
    Stream* _debugStream;

    // Command Buffering
    RingBuffer<uint16_t, CMD_BUFFER_SIZE> _cmdFifo;

    // Status Report State
    LMC_Status_t _currentStatus;
    uint32_t _lastStatusTime;

    // Parser State
    enum State {
        WAIT_OPCODE,
        WAIT_DATA_1,
        WAIT_DATA_2,
        WAIT_DATA_3
    };
    State _pState;
    uint16_t _currentOpcode;
    uint16_t _arg1;
    uint16_t _arg2;

    void processIncomingUSB();
    void executeNextCommand();
    void sendStatusReport();
    void log(const char* format, ...); // Internal helper
};

#endif