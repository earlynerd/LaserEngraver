#include "LMCV4Driver.h"

// Constructor
LMCV4Driver::LMCV4Driver() {
    state.x = 0x8000;
    state.y = 0x8000;
    _ep_out = 0;
    _ep_in = 0;
}

void LMCV4Driver::begin() {
    TinyUSBDevice.clearConfiguration();
    TinyUSBDevice.setID(LMCV4_VID, LMCV4_PID);
    TinyUSBDevice.setProductDescriptor("USBLMCV4");
    this->setStringDescriptor("USBLMCV4");

    _itfnum = TinyUSBDevice.allocInterface(1);
    TinyUSBDevice.addInterface(*this);
}

uint16_t LMCV4Driver::getInterfaceDescriptor(uint8_t itfnum_deprecated, uint8_t *buf, uint16_t bufsize) {
    (void)itfnum_deprecated;

    if (buf) {
        while (_ep_out != 0x02) _ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
        while (_ep_in != 0x88)  _ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    }
    
    // Standard Interface Descriptor + 2 Endpoint Descriptors
    uint8_t const desc[] = { TUD_VENDOR_DESCRIPTOR(_itfnum, _strid, _ep_out, _ep_in, 64) };
    uint16_t len = sizeof(desc);
    
    if (bufsize < len) return 0;
    memcpy(buf, desc, len);
    return len;
}

// --------------------------------------------------------------------------
// MAIN LOOPS
// --------------------------------------------------------------------------

void LMCV4Driver::update() {
    // 1. Read Raw USB Data
    if (tud_vendor_n_available(_itfnum) > 0) {
        uint32_t count = tud_vendor_n_read(_itfnum, _rx_buf_temp, sizeof(_rx_buf_temp));
        
        // Push into Ring Buffer to handle fragmented packets
        for(uint32_t i=0; i<count; i++) {
            if(!_usbStreamBuffer.push(_rx_buf_temp[i])) {
                if(_debugStream) _debugStream->println("ERR: USB RX Overflow");
                break; 
            }
        }
    }

    // 2. Process Stream into Commands
    processIncomingStream();
}

void LMCV4Driver::run() {
    // Executes commands from the Job Queue physically
    // This simulates the "Machine" consuming the buffer
    
    if (!_jobQueue.isEmpty()) {
        state.is_running = true; // We are processing data
        
        // TODO: Here you would normally check if the stepper/galvo driver is BUSY.
        // For now, we assume instantaneous execution for logic checking.
        // In a real machine, you would check: if(galvo.isMoving()) return;

        BalorCommand cmd;
        _jobQueue.pop(cmd);
        
        if(_debug) log("EXE", cmd);
        executeCommand(cmd);
    } else {
        // Queue is empty
        // Keep is_running TRUE if we are physically moving (check hardware).
        // For this template, if queue is empty, we are idle.
        state.is_running = false;
    }
}

// --------------------------------------------------------------------------
// COMMAND PROCESSING
// --------------------------------------------------------------------------

void LMCV4Driver::processIncomingStream() {
    // We need at least 12 bytes for a valid command
    while (_usbStreamBuffer.available() >= CMD_SIZE) {
        
        // Peek the command first to decide logic
        uint8_t rawBytes[CMD_SIZE];
        for(int i=0; i<CMD_SIZE; i++) {
            _usbStreamBuffer.peekAt(i, rawBytes[i]);
        }

        BalorCommand* cmd = (BalorCommand*)rawBytes;

        // DECISION: System Command or Job Command?
        if (cmd->opcode < 0x8000) {
            // --- SYSTEM COMMAND (0x00xx) ---
            // Consume immediately from buffer
            BalorCommand sysCmd;
            for(int i=0; i<CMD_SIZE; i++) { 
                uint8_t b; _usbStreamBuffer.pop(b); 
                ((uint8_t*)&sysCmd)[i] = b; 
            }
            
            if(_debug) log("SYS", sysCmd);
            handleSystemCommand(sysCmd);
        
        } else {
            // --- JOB COMMAND (0x8xxx) ---
            // Only consume if we have space in the Job Queue
            if (!_jobQueue.isFull()) {
                BalorCommand jobCmd;
                for(int i=0; i<CMD_SIZE; i++) { 
                    uint8_t b; _usbStreamBuffer.pop(b); 
                    ((uint8_t*)&jobCmd)[i] = b; 
                }
                
                // Push to execution queue
                _jobQueue.push(jobCmd);
            } else {
                // Queue full! Stop processing stream. 
                // This leaves data in _usbStreamBuffer.
                // The system status report will tell Host we are not ready.
                return; 
            }
        }
    }
}

void LMCV4Driver::handleSystemCommand(const BalorCommand& cmd) {
    uint8_t report[REPORT_SIZE] = {0};

    // 1. Gather Hardware State
    uint16_t live_x, live_y;
    hw_getPos(live_x, live_y);
    
    uint16_t inputs = state.port_val;
    // Simulate Laser Bit in port for ReadPort command
    if (state.laser_on) inputs |= 0x100; 
    else inputs &= ~0x100;

    // 2. Determine Status Byte (Byte 6)
    // This is the most critical part for Lightburn/Python driver connection
    uint8_t status = 0;
    
    // READY BIT (0x20): High if we have room in the buffer
    // Lightburn waits for this before sending the next chunk
    if (!_jobQueue.isFull() && _usbStreamBuffer.available() < 2048) {
        status |= LMC_STATUS_READY;
        state.is_ready = true;
    } else {
        state.is_ready = false;
    }

    // RUNNING BIT (0x04): High if we are working
    if (state.is_running) {
        status |= LMC_STATUS_RUNNING;
    }

    // 3. Prepare Specific Responses
    switch (cmd.opcode) {
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

        case 0x0007: // Status Poll / Version
        case 0x0025: // Status Poll
        case 0x0019: // End of List (Commit)
            // These commands mostly care about the Status Byte (Byte 6)
            // Standard "OK" signature often seen in dumps
            report[0] = 0x00; 
            report[1] = 0x00;
            report[2] = 0x21; // Magic bytes seen in python blobs
            report[3] = 0x84;
            report[4] = 0x09;
            report[5] = 0x00;
            // Byte 6 is inserted below
            report[7] = 0x02; 
            break;

        case 0x0005: // Execute
            state.is_running = true;
            break;

        case 0x0012: // Reset / Abort
            state.is_running = false;
            _jobQueue.clear();
            _usbStreamBuffer.clear();
            hw_laserControl(false);
            break;
            
        case 0x0021: // Write Port immediate
             // Typically handled in queue, but some drivers use 0x0021 for immediate IO
             state.port_val = cmd.params[0];
             break;
    }

    // Apply Status
    report[6] = status;

    // 4. Send Response
    // Note: Opcode 0x10 usually doesn't get a response in some logs, but 
    // generic logic usually replies to everything except specific streaming packets.
    tud_vendor_n_write(0, report, REPORT_SIZE);
    tud_vendor_n_flush(0);
}

void LMCV4Driver::executeCommand(const BalorCommand& cmd) {
    switch (cmd.opcode) {
        case 0x8001: // Travel (Jump)
            state.x = cmd.params[1];
            state.y = cmd.params[0];
            hw_travel(state.x, state.y);
            break;
            
        case 0x8005: // Cut (Mark)
            state.x = cmd.params[1];
            state.y = cmd.params[0];
            hw_cut(state.x, state.y);
            break;
            
        case 0x8021: // Laser Control
            state.laser_on = (cmd.params[0] > 0);
            hw_laserControl(state.laser_on);
            break;

        case 0x8012: // Set Power
            hw_setPower(cmd.params[0]);
            break;

        case 0x801B: // Q-Switch / Freq
        case 0x800A: 
            hw_setFrequency(cmd.params[0]);
            break;

        case 0x800C: // Cut Speed
        case 0x8006: // Jump Speed
            hw_setSpeed(cmd.params[0]);
            break;
            
        case 0x8002: // End of List marker
            // This is often a NOP in execution, just marks end of a segment
            break;
            
        // Delays - In a real machine, these would block the 'run' loop non-blocking
        // using millis() or micros(). For simplicity here, we might just wait.
        case 0x800D: // Jump Delay
            state.jump_delay = cmd.params[0];
            delayMicroseconds(state.jump_delay * 10); // Params often 10us units
            break;
        case 0x8007: // Laser On Delay
             state.laser_on_delay = cmd.params[0];
             break;
    }
}

// --------------------------------------------------------------------------
// DEBUGGING
// --------------------------------------------------------------------------

void LMCV4Driver::setDebug(bool enabled, Stream *stream) {
    _debug = enabled;
    _debugStream = stream;
}

void LMCV4Driver::log(const char* prefix, const BalorCommand& cmd)
{
    if (!_debug || !_debugStream) return;
    _debugStream->print(prefix);
    _debugStream->print(" [");
    _debugStream->print(cmd.opcode, HEX);
    _debugStream->print("] ");
    _debugStream->print(getOpcodeName(cmd.opcode));
    _debugStream->println();
}

const char *LMCV4Driver::getOpcodeName(uint16_t op) {
    switch (op) {
        case 0x0005: return "SYS_EXEC";
        case 0x0007: return "SYS_VER/POLL";
        case 0x000C: return "SYS_GETPOS";
        case 0x0012: return "SYS_RESET";
        case 0x0019: return "SYS_COMMIT";
        case 0x0021: return "SYS_WR_PORT";
        case 0x0025: return "SYS_POLL";
        case 0x8001: return "JOB_TRAVEL";
        case 0x8002: return "JOB_END";
        case 0x8005: return "JOB_CUT";
        case 0x8021: return "JOB_LSR_CTL";
        default: return "UNK";
    }
}