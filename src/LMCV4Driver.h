#ifndef LMCV4_DRIVER_H
#define LMCV4_DRIVER_H

#include <Adafruit_TinyUSB.h>
#include "LMCV4_Protocol.h"
#include "RingBuffer.h"

class LMCV4Driver : public Adafruit_USBD_Interface {
public:
    LMCV4Driver();
    void begin();
    
    // Call this in the main loop as fast as possible
    // Handles USB I/O and parsing
    void update(); 

    // Call this in the main loop to execute queued hardware movements
    // Handles the actual laser/galvo control logic
    void run(); 

    void setDebug(bool enabled, Stream* stream = &Serial);

    // TinyUSB Descriptor Hook
    virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t* buf, uint16_t bufsize);

protected:
    // Hardware Abstraction Layer (Override these in main.cpp)
    virtual void hw_travel(uint16_t x, uint16_t y) = 0;
    virtual void hw_cut(uint16_t x, uint16_t y) = 0;
    virtual void hw_laserControl(bool on) = 0;
    virtual void hw_setPower(uint16_t power) = 0;
    virtual void hw_setFrequency(uint16_t period) = 0;
    virtual void hw_setSpeed(uint16_t speed) = 0;
    virtual void hw_getPos(uint16_t& live_x, uint16_t& live_y) = 0;
    virtual uint16_t hw_getInputs() = 0;

    // Internal Machine State
    struct State {
        uint16_t x = 0x8000;
        uint16_t y = 0x8000;
        
        // Logical states
        bool is_ready = true;   // Ready to receive commands
        bool is_running = false; // Currently executing a job list
        
        // Hardware states
        bool laser_on = false;
        uint16_t port_val = 0;
        
        // Timing parameters (buffered from commands)
        uint16_t jump_delay = 0;
        uint16_t mark_delay = 0;
        uint16_t poly_delay = 0;
        uint16_t laser_on_delay = 0;
        uint16_t laser_off_delay = 0;
    } state;

    // USB Buffers
    uint8_t _rx_buf_temp[4096]; // Temporary buffer for Raw USB read
    RingBuffer<uint8_t, 16384> _usbStreamBuffer; // Buffer to re-assemble stream into 12-byte cmds

  
    // Stores parsed commands waiting to be executed by hardware
    RingBuffer<BalorCommand, 512> _jobQueue; 

    // TinyUSB Handles
    uint8_t _ep_out;
    uint8_t _ep_in;
    uint8_t _itfnum;
    
    bool _debug = false;
    Stream* _debugStream = nullptr;

    // Parsing & Processing
    void processIncomingStream();
    void handleJobCommand(const BalorCommand& cmd);
    void handleSystemCommand(const BalorCommand& cmd);
    
    // Execution
    void executeCommand(const BalorCommand& cmd);

    // Utilities
    void log(const char* prefix, const BalorCommand& cmd);
    const char* getOpcodeName(uint16_t op);
};

#endif