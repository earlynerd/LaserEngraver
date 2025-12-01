#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "LMCV4Driver.h"
#include "XY2Galvo.h"
#define SHADOW_BUFFER_SIZE 4096
#define FIELD_SIZE_MM  110.0
#define GALVO_RANGE 65536.0f
#define UPDATE_RATE_HZ 100000.0f // 1/10us

extern LaserSet laser_set[];

class RP2350Laser : public LMCV4Driver
{
private:
    LaserSet _shadowBuffer[SHADOW_BUFFER_SIZE];
    size_t _shadowHead = 0;
    const float SPEED_FACTOR = GALVO_RANGE / (FIELD_SIZE_MM * UPDATE_RATE_HZ);
    // 2. Current State (Pending changes from USB)
    LaserSet _pendingMarkSettings = laser_set[2];    // Holds current power, freq, mark speed for marking
    LaserSet _pendingJumpSettings = laser_set[0];    // Holds current power, freq, mark speed for jumps
    
    LaserSet *commitLaserSet(bool is_marking)
    {
        if(is_marking)
        // Copy pending settings into the persistent buffer
        _shadowBuffer[_shadowHead] = _pendingMarkSettings;
        else 
        _shadowBuffer[_shadowHead] = _pendingJumpSettings;
        // Get address of the persistent copy
        LaserSet *stablePtr = &_shadowBuffer[_shadowHead];

        // Advance head, wrapping around
        _shadowHead = (_shadowHead + 1) % SHADOW_BUFFER_SIZE;
        return stablePtr;
    }

protected:
    void hw_travel(uint16_t x, uint16_t y, XY2Galvo *galvo) override
    {
        // Mirrors move with laser OFF
        float targetX = static_cast<float>(state.x) - 32768.0f;
        float targetY = static_cast<float>(state.y) - 32768.0f;
        LaserSet* useThisSet = commitLaserSet(false);
        galvo->drawTo({targetX, targetY}, (const LaserSet&)*useThisSet);
 
        // Serial1.printf("Jump: %d, %d\n", x, y);
    }

    void hw_cut(uint16_t x, uint16_t y, XY2Galvo *galvo) override
    {
        // Mirrors move with laser ON
        float targetX = static_cast<float>(state.x) - 32768.0f;
        float targetY = static_cast<float>(state.y) - 32768.0f;
        LaserSet* useThisSet = commitLaserSet(true);
        galvo->drawTo({targetX, targetY}, (const LaserSet&)*useThisSet);
        // Serial1.printf("Mark: %d, %d\r\n", x, y);
    }

    void hw_laserControl(bool on, XY2Galvo *galvo) override
    {
        digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
        if (!on)
            galvo->moveTo({static_cast<float>(state.x) - (float)32768.0, static_cast<float>(state.y) - (float)32768.0});
        else
            galvo->drawTo({static_cast<float>(state.x) - (float)32768.0, static_cast<float>(state.y) - (float)32768.0}, laser_set[1]);
        // Serial1.println(on ? "Laser ON" : "Laser OFF");
    }

    void hw_setPower(uint16_t power, XY2Galvo *galvo) override
    {
        uint pattern = 0x03ff >> map(power, 0, 4095, 10, 0);
        _pendingMarkSettings.pattern = pattern;        
        //Serial1.printf("Power: %d\r\n", power);
    }

    void hw_setFrequency(uint16_t period, XY2Galvo *galvo) override {}
    void hw_setMarkSpeed(float speed, XY2Galvo *galvo) override
    {
        float stepPerTick = (float)speed * SPEED_FACTOR;
        _pendingMarkSettings.speed =stepPerTick;
    }
    void hw_setJumpSpeed(float speed, XY2Galvo *galvo) override
    {
        float stepPerTick = (float)speed * SPEED_FACTOR;
        _pendingJumpSettings.speed = stepPerTick;
    }

    void hw_getPos(uint16_t &live_x, uint16_t &live_y, XY2Galvo *galvo) override
    {
        // Report logical position for now
        live_x = state.x;
        live_y = state.y;
    }

    uint16_t hw_getInputs() override
    {
        return 0x0000;
    }

   
    void hw_abort(XY2Galvo *galvo)override {
        _shadowHead = 0;
        galvo->requestAbort();
    }  
    void hw_setPulseWidth(uint16_t us)override {
        state.pulseWidth = us;
    }
    void hw_setLaserOnDelay(uint16_t us)override{
        state.laser_on_delay = us;
        _pendingMarkSettings.delay_a = us/10;
    }
    void hw_setLaserOffDelay(uint16_t us)override{
        state.laser_off_delay = us;
        _pendingMarkSettings.delay_e = us/10;
    }
    void hw_setEndDelay(uint16_t us) override{
        state.end_delay = us;
        
    }
    void hw_setPolygonDelay(uint16_t us)override{
        state.poly_delay = us;
        _pendingMarkSettings.delay_m = us/10;
    }
};

RP2350Laser machine;
XY2Galvo galvo;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    galvo.init();

    galvo.start();
    // Setup Serial for Debugging
    Serial1.begin(1000000);

    machine.setDebug(false, &Serial1);
    machine.begin(&galvo, &laser_queue);

    // Re-enumerate USB
    if (TinyUSBDevice.mounted())
    {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

    Serial1.println("Balor Controller Ready");
}

void loop()
{
    // 1. Handle USB Communication (High Priority)
    // Reads raw data, buffers it, parses commands into the queue
    machine.update();

    // 2. Execute Queued Commands (Machine Priority)
    // Pops commands from queue and drives hardware
    machine.run();
}