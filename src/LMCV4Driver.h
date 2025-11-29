#ifndef LMCV4_DRIVER_H
#define LMCV4_DRIVER_H

#include <Adafruit_TinyUSB.h>

// Balor Protocol Constants
#define LMCV4_VID           0x9588
#define LMCV4_PID           0x9899
#define CMD_SIZE            12
#define REPORT_SIZE         8

// Command Structure
struct BalorCommand {
    uint16_t opcode;
    uint16_t params[5];
} __attribute__((packed));

class LMCV4Driver : public Adafruit_USBD_Interface {
public:
    LMCV4Driver();
    void begin();
    void update();
    void setDebug(bool enabled, Stream* stream = &Serial);

    // TinyUSB Descriptor Hook
    virtual uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t* buf, uint16_t bufsize);

protected:
    // Virtual Hardware Hooks (Override in your main sketch)
    virtual void hw_travel(uint16_t x, uint16_t y) = 0;
    virtual void hw_cut(uint16_t x, uint16_t y) = 0;
    virtual void hw_laserControl(bool on) = 0;
    virtual void hw_setPower(uint16_t power) = 0;
    virtual void hw_setFrequency(uint16_t period) = 0;
    virtual void hw_setSpeed(uint16_t speed) = 0;
    virtual void hw_getPos(uint16_t& live_x, uint16_t& live_y) = 0;
    virtual uint16_t hw_getInputs() = 0;
    struct State {
        uint16_t x = 0x8000;
        uint16_t y = 0x8000;
        bool is_ready = true;
        bool is_running = false;
        bool laser_on = false;
        bool mark_on = false;
        uint16_t port;
    } state;

    uint8_t _buf[5000];
    // Replaced non-existent Endpoint class with raw addresses
    // We will use tud_vendor_read/write() in the implementation
    uint8_t _ep_out;
    uint8_t _ep_in;
    uint8_t _itfnum;
    bool _debug = false;
    Stream* _debugStream = nullptr;

    void processCommand(BalorCommand* cmd);
    void handleJobCommand(BalorCommand* cmd);
    void handleSystemCommand(BalorCommand* cmd);
    void log(const char* prefix, BalorCommand* cmd);
    const char* getOpcodeName(uint16_t op);
};

#endif