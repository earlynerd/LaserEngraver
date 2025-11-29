#include "LaserInterface.h"

// Interface Class 0xFF = Vendor Specific
#define ITF_CLASS_VENDOR  0xFF

LaserInterface::LaserInterface() 
    : Adafruit_USBD_Interface(), _debugStream(nullptr), _pState(WAIT_OPCODE), _ep_in(0), _ep_out(0)
{
    // Initialize status defaults
    memset(&_currentStatus, 0, sizeof(LMC_Status_t));
    _currentStatus.galvoX = 0x7FFF; 
    _currentStatus.galvoY = 0x7FFF;
}

void LaserInterface::setDebugStream(Stream& stream) {
    _debugStream = &stream;
}

void LaserInterface::log(const char* format, ...) {
    if (!_debugStream) return;
    char loc_buf[64];
    va_list args;
    va_start(args, format);
    vsnprintf(loc_buf, sizeof(loc_buf), format, args);
    va_end(args);
    _debugStream->print(loc_buf);
}

bool LaserInterface::begin() {

    TinyUSBDevice.clearConfiguration();
    TinyUSBDevice.setManufacturerDescriptor("BJJCZ");
    TinyUSBDevice.setProductDescriptor("LMCV4-FIBER-M Compatible");
    TinyUSBDevice.setID(LMC_VID, LMC_PID);

    // Allocate Interface. 
    // IMPORTANT: This might return 0, 1, or higher depending on what other classes (DFU/MSC) are active.
    _itf = TinyUSBDevice.allocInterface(1);


    
    log("Init: ITF=%d EP_OUT=0x%02X EP_IN=0x%02X\n", _itf, _ep_out, _ep_in);

    return TinyUSBDevice.addInterface(*this);
}

uint16_t LaserInterface::getInterfaceDescriptor(uint8_t none, uint8_t* buf, uint16_t bufsize) {
    (void)none;
    uint16_t desc_len = 9 + 7 + 7;
    if (!buf || bufsize < desc_len) return 0;

    uint8_t desc_interface[] = { 9, TUSB_DESC_INTERFACE, _itf, 0, 2, ITF_CLASS_VENDOR, 0x00, 0x00, 0 };
    memcpy(buf, desc_interface, 9);
    buf += 9;

    uint8_t desc_ep_out[] = { 7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0 };
    memcpy(buf, desc_ep_out, 7);
    buf += 7;

    uint8_t desc_ep_in[] = { 7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0 };
    memcpy(buf, desc_ep_in, 7);
    buf += 7;

    return desc_len;
}

void LaserInterface::updateStatus(uint16_t x, uint16_t y, uint16_t gpio) {
    _currentStatus.galvoX = x;
    _currentStatus.galvoY = y;
    _currentStatus.gpioIn = gpio;
}

void LaserInterface::processIncomingUSB() {
    // FIX 1: Use 'tud_vendor_n_available' with specific interface index (_itf)
    if (!tud_vendor_n_available(_itf)) return;

    uint8_t buf[64];
    // FIX 2: Use 'tud_vendor_n_read' with _itf
    uint32_t count = tud_vendor_n_read(_itf, buf, sizeof(buf));
    
    if (count > 0) {
        // Optional: log("RX[%d]: %02X %02X...\n", count, buf[0], buf[1]);

        for (uint32_t i = 0; i < count; i += 2) {
            if (i + 1 < count) {
                uint16_t word = buf[i] | (buf[i+1] << 8);
                if (!_cmdFifo.push(word)) {
                    _currentStatus.statusFlags |= STATUS_ERROR; 
                }
            }
        }
        
        // Immediate response on RX to satisfy any handshake
        sendStatusReport(); 
    }
}

void LaserInterface::sendStatusReport() {
    // 1. Update Busy flag dynamically based on FIFO level
    if (_cmdFifo.availableSpace() < 128) {
        _currentStatus.statusFlags |= STATUS_BUSY;
    } else {
        _currentStatus.statusFlags &= ~STATUS_BUSY;
    }

    // 2. Keep the IN buffer populated
    // We check if there is space for a packet in the USB hardware buffer.
    // If there is space, we fill it immediately. 
    // This ensures that when the host software polls Endpoint 0x81, data is ALREADY there.
    if (tud_vendor_n_write_available(_itf) >= sizeof(LMC_Status_t)) {
        tud_vendor_n_write(_itf, &_currentStatus, sizeof(LMC_Status_t));
    }
}

void LaserInterface::executeNextCommand() {
    if (_cmdFifo.isEmpty()) return;

    uint16_t data;

    switch (_pState) {
        case WAIT_OPCODE:
            if (_cmdFifo.pop(data)) {
                _currentOpcode = data;
                
                switch (_currentOpcode) {
                    case CMD_JUMP_ABS:
                    case CMD_MARK_ABS:
                        _pState = WAIT_DATA_1;
                        break;
                    case CMD_LSR_CTRL:
                    case CMD_SET_PARAM:
                    case CMD_LSR_FREQ:
                    case CMD_LSR_POWER:
                        _pState = WAIT_DATA_1;
                        break;
                    default:
                        // log("Unknown OP: 0x%04X\n", _currentOpcode);
                        break;
                }
            }
            break;

        case WAIT_DATA_1:
            if (_cmdFifo.pop(data)) {
                _arg1 = data;
                if (_currentOpcode == CMD_JUMP_ABS || _currentOpcode == CMD_MARK_ABS) {
                    _pState = WAIT_DATA_2;
                } else {
                    if (_currentOpcode == CMD_LSR_CTRL && onLaserControl) onLaserControl(_arg1);
                    else if (_currentOpcode == CMD_LSR_FREQ && onParamUpdate) onParamUpdate(CMD_LSR_FREQ, _arg1);
                    else if (_currentOpcode == CMD_LSR_POWER && onParamUpdate) onParamUpdate(CMD_LSR_POWER, _arg1);
                    _pState = WAIT_OPCODE;
                }
            }
            break;

        case WAIT_DATA_2:
            if (_cmdFifo.pop(data)) {
                _arg2 = data;
                if (_currentOpcode == CMD_JUMP_ABS && onJumpTo) onJumpTo(_arg1, _arg2);
                else if (_currentOpcode == CMD_MARK_ABS && onMarkTo) onMarkTo(_arg1, _arg2);
                _pState = WAIT_OPCODE;
            }
            break;
            
        case WAIT_DATA_3: break;
    }
}

bool LaserInterface::task() {
    // Check if USB is even mounted
    if (!tud_mounted()) return false;

    processIncomingUSB();

    for (int i=0; i<20; i++) {
        executeNextCommand();
        if (_cmdFifo.isEmpty()) break;
    }

    // Always attempt to top up the Status endpoint
    // We removed the 10ms timer logic.
    sendStatusReport();
    
    return !_cmdFifo.isEmpty();
}