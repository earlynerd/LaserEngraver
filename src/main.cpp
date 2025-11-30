#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "LMCV4Driver.h"
#include "XY2Galvo.h"

extern LaserSet laser_set[];

class RP2350Laser : public LMCV4Driver {
protected:
    void hw_travel(uint16_t x, uint16_t y,  XY2Galvo* galvo) override {
        // Mirrors move with laser OFF
        galvo->moveTo({static_cast<float>(state.x) - (float)32768.0,static_cast<float>(state.y) - (float)32768.0});
        // In a real implementation, this triggers the Galvo DACs
        //Serial1.printf("Jump: %d, %d\n", x, y);
    }

    void hw_cut(uint16_t x, uint16_t y,  XY2Galvo* galvo) override {
        // Mirrors move with laser ON
        galvo->drawTo({static_cast<float>(state.x) - (float)32768.0,static_cast<float>(state.y) - (float)32768.0}, laser_set[1]);
        //Serial1.printf("Mark: %d, %d\r\n", x, y);
    }

    void hw_laserControl(bool on,  XY2Galvo* galvo) override {
        digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
        if(!on)galvo->moveTo({static_cast<float>(state.x) - (float)32768.0,static_cast<float>(state.y) - (float)32768.0});
        else galvo->drawTo({static_cast<float>(state.x) - (float)32768.0,static_cast<float>(state.y) - (float)32768.0}, laser_set[1]);
        //Serial1.println(on ? "Laser ON" : "Laser OFF");
    }

    void hw_setPower(uint16_t power,  XY2Galvo* galvo) override {
        //Serial1.printf("Power: %d\r\n", power);
    }

    void hw_setFrequency(uint16_t period, XY2Galvo* galvo) override {}
    void hw_setSpeed(uint16_t speed, XY2Galvo* galvo) override {}
    
    void hw_getPos(uint16_t& live_x, uint16_t& live_y, XY2Galvo* galvo) override {
        // Report logical position for now
        live_x = state.x;
        live_y = state.y;
    }
    
    uint16_t hw_getInputs() override { 
        return 0x0000; 
    }
};

RP2350Laser machine;
XY2Galvo galvo;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    galvo.init();
  

    galvo.start();
    // Setup Serial for Debugging
    Serial1.begin(1000000); 
    
    machine.setDebug(false, &Serial1);
    machine.begin(&galvo, &laser_queue);

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