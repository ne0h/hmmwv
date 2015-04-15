#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_
#include <stdint.h>

const uint32_t	BAUDRATE		=	115200;
const uint8_t	BUFFER_LENGTH	=	16;
const uint8_t	CMD_LENGTH		=	4;

const uint8_t	DRIVE_LEFT_EN	=	35;
const uint8_t	DRIVE_LEFT_DIR	=	34;
const uint8_t	DRIVE_LEFT_SPD	=	3;
const uint8_t	DRIVE_LEFT_MNT	=	5;	// pin 18

const uint8_t	DRIVE_RIGHT_EN	=	37;
const uint8_t	DRIVE_RIGHT_DIR =	36;
const uint8_t	DRIVE_RIGHT_SPD	=	2;
const uint8_t	DRIVE_RIGHT_MNT	=	4;	// pin 19

const char CMD_SET_DRIVE_LEFT_FORWARD[]		= "sdlf";
const char CMD_SET_DRIVE_LEFT_BACKWARD[]	= "sdlb";
const char CMD_SET_DRIVE_LEFT_STOP[]		= "sdls";

const char CMD_SET_DRIVE_RIGHT_FORWARD[]	= "sdrf";
const char CMD_SET_DRIVE_RIGHT_BACKWARD[]	= "sdrb";
const char CMD_SET_DRIVE_RIGHT_STOP[]		= "sdrs";

#endif
