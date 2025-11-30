#include "LMCV4Driver.h"

// Constructor
LMCV4Driver::LMCV4Driver() {
    state.x = 0x8000;
    state.y = 0x8000;
    _ep_out = 0;
    _ep_in = 0;
}

void LMCV4Driver::begin(XY2Galvo* galvo, LaserQueue* queue) {
    _galvo = galvo;
    _queue = queue;
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
        //state.is_running = true; // We are processing data
        
        // TODO: Here you would normally check if the stepper/galvo driver is BUSY.
        // For now, we assume instantaneous execution for logic checking.
        // In a real machine, you would check: if(galvo.isMoving()) return;
        if(_queue->free() > 1)
        {
        BalorCommand cmd;
        _jobQueue.pop(cmd);
        
        if(_debug) log("EXE", cmd);
        executeCommand(cmd);
        }
        else return;
    } 
    if(_jobQueue.isEmpty()){
        // Queue is empty
        // Keep is_running TRUE if we are physically moving (check hardware).
        // For this template, if queue is empty, we are idle.
        //state.is_running = false;
    }
    //if(_queue->avail()) state.is_running = true;
   // else  state.is_running = false;
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

        if (cmd->opcode < 0x8000) {
            // --- SYSTEM COMMAND (0x00xx) ---
            // Consume immediately from buffer
            BalorCommand sysCmd;
            for(int i=0; i<CMD_SIZE; i++) { 
                uint8_t b; _usbStreamBuffer.pop(b); 
                ((uint8_t*)&sysCmd)[i] = b; 
            }
            
            if((_debug) && (cmd->opcode != 0x07) && (cmd->opcode != 0x25)&& (cmd->opcode != 0x10)) log("SYS", sysCmd);
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
    hw_getPos(live_x, live_y, _galvo);
    
    uint16_t inputs = state.port_val;
    // Simulate Laser Bit in port for ReadPort command
    if (state.laser_on) inputs |= 0x100; 
    else inputs &= ~0x100;

    // 2. Determine Status Byte (Byte 6)
    // This is the most critical part for Lightburn/Python driver connection
    uint8_t status = 0;
    
    // READY BIT (0x20): High if we have room in the buffer
    // Lightburn waits for this before sending the next chunk
    if (!_jobQueue.isFull() && (_queue->free() > 256) && _usbStreamBuffer.available() < (sizeof(_usbStreamBuffer)-CHUNK_SIZE)) {
        status |= LMC_STATUS_READY;
        state.is_ready = true;
    } else {
        state.is_ready = false;
    }

    if(_queue->avail()) state.is_running = true;
    else state.is_running = false;
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

        
        case 0x0025: // Status Poll
            //00 00 12 c0 00 00 xx 02
            report[0] = 0x00; 
            report[1] = 0x00;
            report[2] = 0x12; // Magic bytes seen in python blobs
            report[3] = 0xc0;
            report[4] = 0x09;
            report[5] = 0x00;
            // Byte 6 is inserted below
            report[7] = 0x02; 
            break;

        case 0x0007: // Status Poll / Version
        case 0x0019: // End of List (Commit)
            // These commands mostly care about the Status Byte (Byte 6)
            // 00 00 21 84 09 00 xx 02
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
            while(_queue->avail()) _queue->pop();
            hw_laserControl(false, _galvo);
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
            hw_travel(state.x, state.y, _galvo);
            state.is_running = 1;
            break;            

        case 0x8005: // Cut (Mark)
            state.x = cmd.params[1];
            state.y = cmd.params[0];
            hw_cut(state.x, state.y, _galvo);
            state.is_running = 1;
            break;
            
        case 0x8021: // Laser Control
            state.laser_on = (cmd.params[0] > 0);
            hw_laserControl(state.laser_on, _galvo);
            break;

        case 0x8012: // Set Power
            hw_setPower(cmd.params[0], _galvo);
            break;

        case 0x801B: // Q-Switch / Freq
        case 0x800A: 
            hw_setFrequency(cmd.params[0], _galvo);
            break;

        case 0x800C: // Cut Speed
        case 0x8006: // Jump Speed
            hw_setSpeed(cmd.params[0], _galvo);
            break;
            
        case 0x8002: // End of List marker
            // This is often a NOP in execution, just marks end of a segment
            state.is_running = 0;
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
    _debugStream->printf("%S [0x%04X] %-14S P1:%-6d, P2:%-6d\r\n", prefix, cmd.opcode, getOpcodeName(cmd.opcode),cmd.params[0], cmd.params[1]);
}

const char* LMCV4Driver::getOpcodeName(uint16_t op) {
    switch(op) {
        // --- System Commands ---
        case 0x0005: return "EXEC";
        case 0x0007: return "VER";
        case 0x0009: return "READ_IN";
        case 0x000C: return "GETPOS";
        case 0x0012: return "RST";
        case 0x0021: return "WPORT";
        case 0x0025: return "STAT";

        // --- Job: Motion ---
        case 0x8001: return "JUMP";
        case 0x8002: return "END";
        case 0x8005: return "CUT";
        case 0x8051: return "START";

        // --- Job: Laser & Timing ---
        case 0x8003: return "LASER_ON_PT";
        case 0x8004: return "MARK_END_DLY";
        case 0x8006: return "TRAVEL_SPD";
        case 0x8007: return "LASER_ON_DLY";
        case 0x8008: return "LASER_OFF_DLY";
        case 0x800A: return "FREQ";
        case 0x800B: return "PULSE_WIDTH";
        case 0x800C: return "CUT_SPD";
        case 0x800D: return "JUMP_DLY";
        case 0x800F: return "POLY_DLY";
        case 0x8011: return "WRITE_PORT";
        case 0x8012: return "POWER";
        case 0x801A: return "FLY_EN";
        case 0x801B: return "QSWITCH";
        case 0x801C: return "DIRECT_SW";
        case 0x801D: return "FLY_DLY";
        case 0x8021: return "LSR_CTRL";
        case 0x8023: return "MARK_COUNT";
        case 0x8050: return "JPT_PARAM";
        default: return "UNK";
    }
}
