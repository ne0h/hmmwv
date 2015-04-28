#ifndef SRC_CONSTANTS_HPP_
#define SRC_CONSTANTS_HPP_

#include <stdint.h>

// connection settings
const uint32_t	BAUDRATE		=	115200;
const uint8_t	BUFFER_LENGTH	=	16;
const uint8_t	CMD_LENGTH		=	4;

// pin settings
const uint8_t	DRIVE_LEFT_EN	=	35;
const uint8_t	DRIVE_LEFT_DIR	=	34;
const uint8_t	DRIVE_LEFT_SPD	=	3;

const uint8_t	DRIVE_RIGHT_EN	=	37;
const uint8_t	DRIVE_RIGHT_DIR =	36;
const uint8_t	DRIVE_RIGHT_SPD	=	2;

const uint8_t	ROTATE_LEFT_EN	=	33;
const uint8_t	ROTATE_LEFT_DIR	=	32;
const uint8_t	ROTATE_LEFT_SPD	=	7;

const uint8_t	ROTATE_RIGHT_EN  =	31;
const uint8_t	ROTATE_RIGHT_DIR =	30;
const uint8_t	ROTATE_RIGHT_SPD =	6;

// commands
const char CMD_SET_DRIVE_LEFT_FORWARD[]		= "sdlf";
const char CMD_SET_DRIVE_LEFT_BACKWARD[]	= "sdlb";
const char CMD_SET_DRIVE_LEFT_STOP[]		= "sdls";

const char CMD_SET_DRIVE_RIGHT_FORWARD[]	= "sdrf";
const char CMD_SET_DRIVE_RIGHT_BACKWARD[]	= "sdrb";
const char CMD_SET_DRIVE_RIGHT_STOP[]		= "sdrs";

const char CMD_SET_ROTATE_LEFT_FORWARD[]	= "srlf";
const char CMD_SET_ROTATE_LEFT_BACKWARD[]	= "srlb";
const char CMD_SET_ROTATE_LEFT_STOP[]		= "srls";

const char CMD_SET_ROTATE_RIGHT_FORWARD[]	= "srrf";
const char CMD_SET_ROTATE_RIGHT_BACKWARD[]	= "srrb";
const char CMD_SET_ROTATE_RIGHT_STOP[]		= "srrs";

#endif
