#include "Crsf.h"

CrsfSerial::CrsfSerial(HardwareSerial &port, uint32_t baud) :
    _port(port), _crc(0xd5), _baud(baud),
    _lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false),
    _passthroughBaud(0)
{}
