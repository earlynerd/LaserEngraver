#include "LMCV4Driver.h"

// Constructor: Initialize Interface with 1 interface count
LMCV4Driver::LMCV4Driver(){
    state.x = 0x8000;
    state.y = 0x8000;
    _ep_out = 0;
    _ep_in = 0;
    // Set Interface String
    //setStringDescriptor("LMCV4 Control");
}

void LMCV4Driver::begin() {
    // Set Device ID
    TinyUSBDevice.clearConfiguration();
    TinyUSBDevice.setID(LMCV4_VID, LMCV4_PID);
    //TinyUSBDevice.setManufacturerDescriptor("BJJCZ");
    TinyUSBDevice.setProductDescriptor("USBLMCV4");
    
    // Set Interface String
    this->setStringDescriptor("USBLMCV4");
    
    // Initialize Base Interface (allocates Interface Number)
    //Adafruit_USBD_Interface::begin();
    _itfnum = TinyUSBDevice.allocInterface(1);
    _debugStream->printf("Allocated interface: %d\r\n", _itfnum);
    bool success = TinyUSBDevice.addInterface(*this);
    if(success) Serial1.println("USB Interface configured!");
    else Serial1.println("Error during USB interface init.");

    // ALLOCATE ENDPOINTS MANUALLY
    // We request a Bulk OUT and Bulk IN endpoint from the stack.
    // The stack returns the address (e.g., 0x01, 0x81).
    
}

// This is called by TinyUSB to generate the Configuration Descriptor
uint16_t LMCV4Driver::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t* buf, uint16_t bufsize) {
    (void) itfnum_deprecated;

    if (buf) {
    while(_ep_out != 0x02) _ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
    _debugStream->printf("Allocated endpoint out: %d\r\n", _ep_out);
    while(_ep_in != 0x88) _ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    _debugStream->printf("Allocated endpoint in: %d\r\n", _ep_in);
   
  }
    // 9 bytes Interface + 7 bytes EP OUT + 7 bytes EP IN = 23 bytes
    uint8_t const desc[] = { TUD_VENDOR_DESCRIPTOR(_itfnum, _strid, _ep_out, _ep_in, 64) };
    
    //uint8_t desc[] = {
        // Interface Descriptor
        // Class 0xFF (Vendor Specific) ensures tud_vendor driver picks this up
        //0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _strid,
        //0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _strid,
        
        // Endpoint Descriptor (OUT)
        // Use the raw address stored in _ep_out
        //0x07, TUSB_DESC_ENDPOINT, 0x01, TUSB_XFER_BULK, lowByte(512), highByte(512), 0x00,
        
        // Endpoint Descriptor (IN)
        // Use the raw address stored in _ep_in
        //0x07, TUSB_DESC_ENDPOINT, 0x81, TUSB_XFER_BULK, lowByte(512), highByte(512), 0x00
    //};
    _debugStream->print("USB device Descriptor: 0x");
    for(int i = 0; i < sizeof(desc); i++)
    {
        if(desc[i] < 0x10) Serial.print("0");
        _debugStream->print(desc[i], HEX);
    }
    _debugStream->println();
    uint16_t len = sizeof(desc);
    if (bufsize < len) return 0;
    memcpy(buf, desc, len);
    return len;
}

void LMCV4Driver::update() {
    // Use the low-level tud_vendor API to check for data.
    // We assume we are the 0-th Vendor interface (standard for single-interface devices).
    uint32_t b = tud_vendor_n_available(_itfnum);
    if (b >= CMD_SIZE) {
        //_debugStream->printf("Packet RX: %ld bytes\r\n", b);
        uint8_t buffer[CMD_SIZE];
        uint32_t count = tud_vendor_n_read(_itfnum, buffer, CMD_SIZE);
        
        if (count == CMD_SIZE) {
            BalorCommand* cmd = (BalorCommand*)buffer;
            processCommand(cmd);
        }
    }
}

void LMCV4Driver::processCommand(BalorCommand* cmd) {
    if (cmd->opcode >= 0x8000) {
        log("JOB", cmd);
        handleJobCommand(cmd);
    } else {
        // Filter high-frequency polling commands from logs
        if (cmd->opcode != 0x0025 && cmd->opcode != 0x0007) {
            log("SYS", cmd);
        }
        handleSystemCommand(cmd);
    }
}

void LMCV4Driver::handleJobCommand(BalorCommand* cmd) {
    switch (cmd->opcode) {
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
        case 0x801B: case 0x800A: // Frequency
            hw_setFrequency(cmd->params[0]);
            break;
        case 0x800C: case 0x8006: // Speed
            hw_setSpeed(cmd->params[0]);
            break;
        case 0x8051: // ReadyMark
            state.is_running = true;
            state.is_ready = false;
            break;
        case 0x8002: // EndOfList
            state.is_running = false;
            state.is_ready = true;
            break;
    }
}

void LMCV4Driver::handleSystemCommand(BalorCommand* cmd) {
    uint8_t report[REPORT_SIZE] = {0};

    // Status Byte (Index 6)
    // 0x20 = Ready, 0x04 = Busy
    uint8_t status = 0;
    if (state.is_ready) status |= 0x20;
    if (state.is_running) status |= 0x04;
    report[6] = status;

    switch (cmd->opcode) {
        case 0x000C: // Get Pos
            report[0] = state.x & 0xFF;
            report[1] = state.x >> 8;
            report[2] = state.y & 0xFF;
            report[3] = state.y >> 8;
            break;
        case 0x0005: // Execute
            state.is_running = true;
            state.is_ready = false; 
            break;
        case 0x0012: // Reset
            state.is_running = false;
            state.is_ready = true;
            hw_laserControl(false);
            break;
    }

    // Send reply using low-level Vendor API
    uint32_t ret = tud_vendor_n_write(_itfnum, report, REPORT_SIZE);
    //_debugStream->printf("TX Packet, %ld bytes sent\r\n", ret);
    ret = tud_vendor_n_flush(_itfnum); // Ensure packet is sent immediately
    //_debugStream->printf("TX flush, %ld bytes flushed\r\n", ret);
}

void LMCV4Driver::setDebug(bool enabled, Stream* stream) {
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
void LMCV4Driver::log(const char* prefix, BalorCommand* cmd) {
    if (!_debug || !_debugStream) return;
    _debugStream->print(prefix); 
    _debugStream->print(" ["); 
    _debugStream->print(cmd->opcode, HEX);
    _debugStream->print("] "); 
    
    _debugStream->print(getOpcodeName(cmd->opcode));
    
    // Custom Parameter Labeling
    switch(cmd->opcode) {
        case 0x8001: // Travel
        case 0x8005: // Cut
            // Balor sends Y then X. We label them correctly here.
            _debugStream->print(" X:"); _debugStream->print(cmd->params[1]);
            _debugStream->print(" Y:"); _debugStream->print(cmd->params[0]);
            break;

        case 0x8006: // Travel Speed
        case 0x800C: // Cut Speed
            _debugStream->print(" Speed:"); _debugStream->print(cmd->params[0]);
            break;

        case 0x8012: // Power
            _debugStream->print(" PwrRatio:"); _debugStream->print(cmd->params[0]);
            break;

        case 0x8021: // Laser Control
            _debugStream->print(" Laser:"); _debugStream->print(cmd->params[0] ? "ON" : "OFF");
            break;

        case 0x801B: // QSwitch
        case 0x800A: // Freq
            _debugStream->print(" FreqVal:"); _debugStream->print(cmd->params[0]);
            break;

        case 0x8007: // On Delay
        case 0x8008: // Off Delay
        case 0x800D: // Jump Delay
        case 0x800F: // Poly Delay
        case 0x8004: // Mark End Delay
        case 0x801D: // Fly Delay
            _debugStream->print(" Delay:"); _debugStream->print(cmd->params[0]);
            // Jump delay (0x800D) sometimes has a second param
            if (cmd->opcode == 0x800D) {
                 _debugStream->print(" P2:"); _debugStream->print(cmd->params[1]);
            }
            break;

        case 0x0021: // Write Port (System)
            _debugStream->print(" PortVal:"); _debugStream->print(cmd->params[0]);
            break;

        default:
            // Fallback for configuration/unknown commands
            _debugStream->print(" P:");
            for(int i=0; i<3; i++) {
                _debugStream->print(cmd->params[i], HEX);
                if(i<2) _debugStream->print(",");
            }
    }
    _debugStream->println();
}

const char* LMCV4Driver::getOpcodeName(uint16_t op) {
    switch(op) {
        // --- System Commands ---
        case 0x0005: return "SYS_EXEC";
        case 0x0007: return "SYS_VER";
        case 0x0009: return "SYS_READ_IN";
        case 0x000C: return "SYS_POS";
        case 0x0012: return "SYS_RST";
        case 0x0021: return "SYS_PORT";
        case 0x0025: return "SYS_STAT";

        // --- Job: Motion ---
        case 0x8001: return "JOB_TRAVEL";
        case 0x8002: return "JOB_END";
        case 0x8005: return "JOB_CUT";
        case 0x8051: return "JOB_RDY";

        // --- Job: Laser & Timing ---
        case 0x8003: return "JOB_LASER_ON_PT";
        case 0x8004: return "JOB_MARK_END_DLY";
        case 0x8006: return "JOB_TRAVEL_SPD";
        case 0x8007: return "JOB_LASER_ON_DLY";
        case 0x8008: return "JOB_LASER_OFF_DLY";
        case 0x800A: return "JOB_FREQ";
        case 0x800B: return "JOB_PULSE_WIDTH";
        case 0x800C: return "JOB_CUT_SPD";
        case 0x800D: return "JOB_JUMP_DLY";
        case 0x800F: return "JOB_POLY_DLY";
        case 0x8011: return "JOB_WRITE_PORT";
        case 0x8012: return "JOB_POWER";
        case 0x801A: return "JOB_FLY_EN";
        case 0x801B: return "JOB_QSWITCH";
        case 0x801C: return "JOB_DIRECT_SW";
        case 0x801D: return "JOB_FLY_DLY";
        case 0x8021: return "JOB_LSR_CTRL";
        case 0x8023: return "JOB_MARK_COUNT";
        case 0x8050: return "JOB_JPT_PARAM";
        
        default: return "UNK";
    }
}