#pragma once

// Mostly taken from here: https://github.com/CapnBry/CRServoF/blob/main/lib/CrsfSerial/crsf_protocol.h


#include "stdint.h"

#if !defined(PACKED)
#define PACKED __attribute__((packed))
#endif

#define CRSF_BAUDRATE           420000
#define CRSF_NUM_CHANNELS 16

