#pragma once
#include <Arduino.h>

#include "crsf_protocol.h"


// CRSF protocol state machine
enum crsf_state {
	CRSF_STATE_WAIT_SYNC,
	CRSF_STATE_WAIT_LENGTH,
	CRSF_STATE_PAYLOAD,
	// CRFS_STATE_END
};

// void rxParseByte(uint8_t c);

class Crsf
{
public:
	Crsf(HardwareSerial &port, uint32_t baud = CRSF_BAUDRATE);

};
