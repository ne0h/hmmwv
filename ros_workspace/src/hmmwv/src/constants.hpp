#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <inttypes.h>

const float     MAX_DRV_SPEED       = 1.642;    // m/s
const float     MAX_TURN_SPEED      = 1.732;    // rad/s
const float     MAX_ROT_SPEED       = 1;        // rad/s

const double    WHEEL_DIAMETER	    = 0.155;
const double    WHEEL_DISTANCE	    = 0.52;
const double    WHEEL_REDUCTION	    = 86.0;

const uint8_t   CMD_LEFT_STOP       = 0x00;
const uint8_t   CMD_RIGHT_STOP      = 0x01;
const uint8_t   CMD_LEFT_DRIVE      = 0x10;
const uint8_t   CMD_RIGHT_DRIVE     = 0x11;

#endif
