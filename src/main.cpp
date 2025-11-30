#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "LMCV4Driver.h"

class RP2350Laser : public LMCV4Driver {
protected:
    void hw_travel(uint16_t x, uint16_t y) override {
        // Mirrors move with laser OFF
        // In a real implementation, this triggers the Galvo DACs
        Serial1.printf("Jump: %d, %d\n", x, y);
    }

    void hw_cut(uint16_t x, uint16_t y) override {
        // Mirrors move with laser ON
        Serial1.printf("Mark: %d, %d\n", x, y);
    }

    void hw_laserControl(bool on) override {
        digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
        Serial1.println(on ? "Laser ON" : "Laser OFF");
    }

    void hw_setPower(uint16_t power) override {
        Serial1.printf("Power: %d\n", power);
    }

    void hw_setFrequency(uint16_t period) override {}
    void hw_setSpeed(uint16_t speed) override {}
    
    void hw_getPos(uint16_t& live_x, uint16_t& live_y) override {
        // Report logical position for now
        live_x = state.x;
        live_y = state.y;
    }
    
    uint16_t hw_getInputs() override { 
        return 0x0000; 
    }
};

RP2350Laser machine;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Setup Serial for Debugging
    Serial1.begin(1000000); 
    
    machine.setDebug(true, &Serial1);
    machine.begin();

    // Re-enumerate USB
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }
    
    Serial1.println("Balor Controller Ready");
}

void loop() {
    // 1. Handle USB Communication (High Priority)
    // Reads raw data, buffers it, parses commands into the queue
    machine.update();

    // 2. Execute Queued Commands (Machine Priority)
    // Pops commands from queue and drives hardware
    machine.run();
}