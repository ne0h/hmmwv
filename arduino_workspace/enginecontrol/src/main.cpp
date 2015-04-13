#include <Arduino.h>

#include "constants.h"

char buffer[BUFFER_LENGTH];
uint8_t buffer_pointer;
bool cmd_available;

void uart_prints(char input[], const uint8_t length) {
	uint8_t i = 0;
	while (i < length) {
		Serial.print(input[i]);
		i++;
	}

	Serial.print('\n');
}

void cmd() {
	char cmd[CMD_LENGTH];
	memcpy(cmd, buffer, CMD_LENGTH);

	/**
	 * drive engine left side
	 */

	// forward
	if (strncmp(cmd, CMD_SET_DRIVE_LEFT_FORWARD, CMD_LENGTH) == 0) {
		digitalWrite(DRIVE_LEFT_EN, HIGH);
		digitalWrite(DRIVE_LEFT_DIR, HIGH);
		analogWrite(DRIVE_LEFT_SPD, buffer[3]);

	// backward
	} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_BACKWARD, CMD_LENGTH) == 0) {
		digitalWrite(DRIVE_LEFT_EN, HIGH);
		digitalWrite(DRIVE_LEFT_DIR, LOW);
		analogWrite(DRIVE_LEFT_SPD, buffer[3]);

	// stop
	} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_STOP, 4) == 0) {
		digitalWrite(DRIVE_LEFT_EN, LOW);

	/**
	 * drive engine right side
	 */

	// forward
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_FORWARD, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, HIGH);
		digitalWrite(DRIVE_RIGHT_DIR, HIGH);
		analogWrite(DRIVE_RIGHT_SPD, buffer[3]);

	// backward
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_BACKWARD, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, HIGH);
		digitalWrite(DRIVE_RIGHT_DIR, LOW);
		analogWrite(DRIVE_RIGHT_SPD, buffer[3]);

	// stop
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_STOP, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, LOW);

	/**
	 * wrong cmd
	 */
	} else {
		Serial.print("invalid commands");
		uart_prints(cmd, CMD_LENGTH);
	}
}

void setup() {
	Serial.begin(BAUDRATE);
	buffer_pointer = 0;
	cmd_available  = false;

	pinMode(DRIVE_LEFT_EN,   OUTPUT);
	pinMode(DRIVE_LEFT_DIR,  OUTPUT);
	pinMode(DRIVE_RIGHT_EN,  OUTPUT);
	pinMode(DRIVE_RIGHT_DIR, OUTPUT);

	// init all output pins with 0

	digitalWrite(DRIVE_LEFT_EN, LOW);
	digitalWrite(DRIVE_LEFT_DIR, LOW);
	analogWrite(DRIVE_LEFT_SPD, 0);

	digitalWrite(DRIVE_RIGHT_EN, LOW);
	digitalWrite(DRIVE_RIGHT_DIR, LOW);
	analogWrite(DRIVE_RIGHT_SPD, 0);
}

void loop() {

	if (cmd_available) {
		cmd_available = false;
		cmd();
	}

}

void serialEvent() {
	while (Serial.available()) {
		const char c = Serial.read();
		buffer[buffer_pointer] = c;

		if (c == '\n') {
			cmd();
			buffer_pointer = 0;
		} else {
			buffer_pointer++;
		}

		// check for buffer overflows
		if (buffer_pointer >= BUFFER_LENGTH) {
			buffer_pointer = 0;
			cmd_available  = true;
		}
	}
}
