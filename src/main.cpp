#include <Adafruit_TinyUSB.h>

#include "LMCV4Driver.h"


class RP2350Laser : public LMCV4Driver {
protected:
    void hw_travel(uint16_t x, uint16_t y) override {
        // TODO: Move mirrors to X, Y (Laser OFF)
    }

    void hw_cut(uint16_t x, uint16_t y) override {
        // TODO: Move mirrors to X, Y (Laser ON)
}

    void hw_laserControl(bool on) override {
        digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
    }

    void hw_setPower(uint16_t power) override {
        // Power 0-4096
}

    void hw_setFrequency(uint16_t period) override {}
    void hw_setSpeed(uint16_t speed) override {}
};

RP2350Laser machine;


void setup() {
    Serial1.begin(2000000);
    Serial1.println("LMCV4 Interface test");
    machine.setDebug(false, &Serial1);
    machine.begin();
    // while(!Serial) delay(10); // Uncomment to wait for Serial monitor
    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
    }

}

void loop() {
machine.update();
}
