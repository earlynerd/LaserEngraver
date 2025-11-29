#include "LMCV4Driver.h"

// Constructor: Initialize Interface with 1 interface count
LMCV4Driver::LMCV4Driver()
{
    state.x = 0x8000;
    state.y = 0x8000;
    _ep_out = 0;
    _ep_in = 0;
    // Set Interface String
    // setStringDescriptor("LMCV4 Control");
}

void LMCV4Driver::begin()
{
    // Set Device ID
    TinyUSBDevice.clearConfiguration();
    TinyUSBDevice.setID(LMCV4_VID, LMCV4_PID);
    // TinyUSBDevice.setManufacturerDescriptor("BJJCZ");
    TinyUSBDevice.setProductDescriptor("USBLMCV4");
     
    // Set Interface String
    this->setStringDescriptor("USBLMCV4");

    // Initialize Base Interface (allocates Interface Number)
    // Adafruit_USBD_Interface::begin();
    _itfnum = TinyUSBDevice.allocInterface(1);
    _debugStream->printf("Allocated interface: %d\r\n", _itfnum);
    bool success = TinyUSBDevice.addInterface(*this);
    if (success)
        Serial1.println("USB Interface configured!");
    else
        Serial1.println("Error during USB interface init.");

    // ALLOCATE ENDPOINTS MANUALLY
    // We request a Bulk OUT and Bulk IN endpoint from the stack.
    // The stack returns the address (e.g., 0x01, 0x81).
}

// This is called by TinyUSB to generate the Configuration Descriptor
uint16_t LMCV4Driver::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t *buf, uint16_t bufsize)
{
    (void)itfnum_deprecated;

    if (buf)
    {
        while (_ep_out != 0x02)
            _ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
        _debugStream->printf("Allocated endpoint out: %d\r\n", _ep_out);
        while (_ep_in != 0x88)
            _ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
        _debugStream->printf("Allocated endpoint in: %d\r\n", _ep_in);
    }
    // 9 bytes Interface + 7 bytes EP OUT + 7 bytes EP IN = 23 bytes
    uint8_t const desc[] = {TUD_VENDOR_DESCRIPTOR(_itfnum, _strid, _ep_out, _ep_in, 64)};
    // uint8_t desc[] = {
    //  Interface Descriptor (Class 0xFF = Vendor)
    //   0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _strid,
    // Endpoint OUT
    //  0x07, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_BULK, lowByte(512), highByte(512), 0x00,
    // Endpoint IN
    //  0x07, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_BULK, lowByte(512), highByte(512), 0x00
    //};

    // uint8_t desc[] = {
    //  Interface Descriptor
    //  Class 0xFF (Vendor Specific) ensures tud_vendor driver picks this up
    // 0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _strid,
    // 0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _strid,

    // Endpoint Descriptor (OUT)
    // Use the raw address stored in _ep_out
    // 0x07, TUSB_DESC_ENDPOINT, 0x01, TUSB_XFER_BULK, lowByte(512), highByte(512), 0x00,

    // Endpoint Descriptor (IN)
    // Use the raw address stored in _ep_in
    // 0x07, TUSB_DESC_ENDPOINT, 0x81, TUSB_XFER_BULK, lowByte(512), highByte(512), 0x00
    //};
    _debugStream->print("USB device Descriptor: 0x");
    for (int i = 0; i < sizeof(desc); i++)
    {
        if (desc[i] < 0x10)
            Serial.print("0");
        _debugStream->print(desc[i], HEX);
    }
    _debugStream->println();
    uint16_t len = sizeof(desc);
    if (bufsize < len)
        return 0;
    memcpy(buf, desc, len);
    return len;
}

void LMCV4Driver::update()
{
    // Use the low-level tud_vendor API to check for data.
    // We assume we are the 0-th Vendor interface (standard for single-interface devices).
    uint32_t b = tud_vendor_n_available(_itfnum);
    //state.is_ready = true;
    if (b >= CMD_SIZE)
    {
        //_debugStream->printf("Packet RX: %ld bytes\r\n", b);
        // uint8_t buffer[b];
        uint32_t count = tud_vendor_n_read(_itfnum, _buf, sizeof(_buf));
        uint32_t idx = 0;
        for (int i = 0; i < count; i += 12)
        {
            BalorCommand cmd;
            memcpy(&cmd, _buf + i, 12);
            processCommand(&cmd);
        }
    }
}

void LMCV4Driver::processCommand(BalorCommand *cmd)
{
    if (cmd->opcode >= 0x8000)
    {
        log("JOB", cmd);
        handleJobCommand(cmd);
    }
    else
    {
        // Filter high-frequency polling commands from logs
        if (cmd->opcode != 0x0025 && cmd->opcode != 0x0007 && cmd->opcode != 0x0009)
        {
            log("SYS", cmd);
        }
        handleSystemCommand(cmd);
    }
}

void LMCV4Driver::handleJobCommand(BalorCommand *cmd)
{
    switch (cmd->opcode)
    {
    case 0x8001: // OpTravel
        hw_travel(cmd->params[1], cmd->params[0]);
        state.x = cmd->params[1];
        state.y = cmd->params[0];
        break;
    case 0x8005: // OpCut
        hw_cut(cmd->params[1], cmd->params[0]);
        state.x = cmd->params[1];
        state.y = cmd->params[0];
        break;
    case 0x8021: // Laser Control
        hw_laserControl(cmd->params[0] > 0);
        break;
    case 0x8012: // Power
        hw_setPower(cmd->params[0]);
        break;
    case 0x801B:
    case 0x800A: // Frequency
        hw_setFrequency(cmd->params[0]);
        break;
    case 0x800C:
    case 0x8006: // Speed
        hw_setSpeed(cmd->params[0]);
        break;
    case 0x8051: // ReadyMark
        state.is_running = true;
        //state.is_ready = false;
        break;
    case 0x8002: // EndOfList
        state.is_running = false;
        state.is_ready = true;
        break;
    }
}

void LMCV4Driver::handleSystemCommand(BalorCommand *cmd)
{
    uint8_t report[REPORT_SIZE] = {0};

    // 1. GET FRESH DATA
    uint16_t live_x, live_y;
    hw_getPos(live_x, live_y);
    // uint16_t inputs = hw_getInputs();
    uint16_t inputs = state.port;
    if (state.laser_on)
        inputs |= 0x100;
    else
        inputs &= (!0x100);
    // 2. CONSTRUCT STATUS BYTE (Byte 6)
    // This is the critical "Heartbeat" of the system.
    uint8_t status = 0;
    if (state.is_ready)
        status |= 0x20;
    if (state.is_running)
        status |= 0x04;
    report[6] = status;

    // 3. HANDLE SPECIFIC COMMAND RESPONSES
    switch (cmd->opcode)
    {
    case 0x000C: // Get Position
        report[0] = live_x & 0xFF;
        report[1] = live_x >> 8;
        report[2] = live_y & 0xFF;
        report[3] = live_y >> 8;
        break;

    case 0x0009: // Read Input Port
        report[0] = inputs & 0xFF;
        report[1] = inputs >> 8;
        break;

    case 0x0007: // Get Version / Check Ready
                 // 00 00 21 84 09 00 24 02
        // often checked to confirm connection and retrieve status
        report[0] = 0x00;
        report[1] = 0x00;
        report[2] = 0x21;
        report[3] = 0x84;
        report[4] = 0x09;
        report[5] = 0x00;
        // report[6] = 0x24;
        report[7] = 0x02;
        break;

    case 0x0019: // Set End Of List (Commit)
        report[0] = 0x00;
        report[1] = 0x00;
        report[2] = 0x21;
        report[3] = 0x84;
        report[4] = 0x09;
        report[5] = 0x00;
        // report[6] = 0x24;
        report[7] = 0x02;
        // The status byte (Byte 6) calculated above is the important part here.
        break;

    case 0x0005: // Execute List
        state.is_running = true;
        state.is_ready = false;
        break;

    case 0x0012: // Reset List
        state.is_running = false;
        state.is_ready = true;
        hw_laserControl(false);
        break;

    case 0x0025:
        // 00 00 12 c0 00 00 24 02
        report[0] = 0x00;
        report[1] = 0x00;
        report[2] = 0x12;
        report[3] = 0xc0;
        report[4] = 0x00;
        report[5] = 0x00;
        report[7] = 0x02;
    }

    // 4. WRITE RESPONSE
    if (cmd->opcode != 0x10)
    {
        tud_vendor_n_write(0, report, REPORT_SIZE);
        tud_vendor_n_flush(0);
    }
}

void LMCV4Driver::setDebug(bool enabled, Stream *stream)
{
    _debug = enabled;
    _debugStream = stream;
}
/*
void LMCV4Driver::log(const char* prefix, BalorCommand* cmd) {
    if (!_debug || !_debugStream) return;
    _debugStream->print(prefix);
    _debugStream->print(" [");
    if(cmd->opcode < 0x10) _debugStream->print("0");
    _debugStream->print(cmd->opcode, HEX);
    _debugStream->print("] ");
    _debugStream->print(getOpcodeName(cmd->opcode));
    _debugStream->print(" P:");
    if(cmd->params[0] < 0x1000) _debugStream->print("0");
    if(cmd->params[0] < 0x100) _debugStream->print("0");
    if(cmd->params[0] < 0x10) _debugStream->print("0");
    _debugStream->print(cmd->params[0], HEX);
    _debugStream->print(",");
    if(cmd->params[1] < 0x1000) _debugStream->print("0");
    if(cmd->params[1] < 0x100) _debugStream->print("0");
    if(cmd->params[1] < 0x10) _debugStream->print("0");
    _debugStream->println(cmd->params[1], HEX);
}
*/
void LMCV4Driver::log(const char *prefix, BalorCommand *cmd)
{
    if (!_debug || !_debugStream)
        return;
    _debugStream->print(prefix);
    _debugStream->print(" [");
    _debugStream->print(cmd->opcode, HEX);
    _debugStream->print("] ");

    _debugStream->print(getOpcodeName(cmd->opcode));

    // Custom Parameter Labeling
    switch (cmd->opcode)
    {
    case 0x8001: // Travel
    case 0x8005: // Cut
        _debugStream->print(" X:");
        _debugStream->print(cmd->params[1]);
        _debugStream->print(" Y:");
        _debugStream->print(cmd->params[0]);
        break;

    case 0x8006: // Travel Speed
    case 0x800C: // Cut Speed
        _debugStream->print(" Speed:");
        _debugStream->print(cmd->params[0]);
        break;

    case 0x8012: // Power
        _debugStream->print(" PwrRatio:");
        _debugStream->print(cmd->params[0]);
        break;

    case 0x8021: // Laser Control
        _debugStream->print(" Laser:");
        _debugStream->print(cmd->params[0] ? "ON" : "OFF");
        break;

    case 0x801B: // QSwitch
    case 0x800A: // Freq
        _debugStream->print(" FreqVal:");
        _debugStream->print(cmd->params[0]);
        break;

    case 0x8007: // On Delay
    case 0x8008: // Off Delay
    case 0x800D: // Jump Delay
    case 0x800F: // Poly Delay
    case 0x8004: // Mark End Delay
    case 0x801D: // Fly Delay
        _debugStream->print(" Delay:");
        _debugStream->print(cmd->params[0]);
        if (cmd->opcode == 0x800D)
        {
            _debugStream->print(" P2:");
            _debugStream->print(cmd->params[1]);
        }
        break;

    case 0x0021: // Write Port (System)
        _debugStream->print(" PortVal:");
        _debugStream->print(cmd->params[0]);
        state.port = cmd->params[0];
        break;

    default:
        _debugStream->print(" P:");
        for (int i = 0; i < 3; i++)
        {
            _debugStream->print(cmd->params[i], HEX);
            if (i < 2)
                _debugStream->print(",");
        }
    }
    _debugStream->println();
}

const char *LMCV4Driver::getOpcodeName(uint16_t op)
{
    switch (op)
    {
    // --- System Commands ---
    case 0x0005:
        return "SYS_EXEC";
    case 0x0007:
        return "SYS_VER";
    case 0x0009:
        return "SYS_READ_IN";
    case 0x000C:
        return "SYS_POS";
    case 0x0012:
        return "SYS_RST";
    case 0x0019:
        return "SYS_END_LIST";
    case 0x0021:
        return "SYS_PORT";
    case 0x0025:
        return "SYS_STAT";

    // --- Job: Motion ---
    case 0x8001:
        return "JOB_TRAVEL";
    case 0x8002:
        return "JOB_END";
    case 0x8005:
        return "JOB_CUT";
    case 0x8051:
        return "JOB_RDY";

    // --- Job: Laser & Timing ---
    case 0x8003:
        return "JOB_LASER_ON_PT";
    case 0x8004:
        return "JOB_MARK_END_DLY";
    case 0x8006:
        return "JOB_TRAVEL_SPD";
    case 0x8007:
        return "JOB_LASER_ON_DLY";
    case 0x8008:
        return "JOB_LASER_OFF_DLY";
    case 0x800A:
        return "JOB_FREQ";
    case 0x800B:
        return "JOB_PULSE_WIDTH";
    case 0x800C:
        return "JOB_CUT_SPD";
    case 0x800D:
        return "JOB_JUMP_DLY";
    case 0x800F:
        return "JOB_POLY_DLY";
    case 0x8011:
        return "JOB_WRITE_PORT";
    case 0x8012:
        return "JOB_POWER";
    case 0x801A:
        return "JOB_FLY_EN";
    case 0x801B:
        return "JOB_QSWITCH";
    case 0x801C:
        return "JOB_DIRECT_SW";
    case 0x801D:
        return "JOB_FLY_DLY";
    case 0x8021:
        return "JOB_LSR_CTRL";
    case 0x8023:
        return "JOB_MARK_COUNT";
    case 0x8050:
        return "JOB_JPT_PARAM";

    default:
        return "UNK";
    }
}