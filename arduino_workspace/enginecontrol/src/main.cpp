#include <Arduino.h>
#include "constants.hpp"

char buffer[BUFFER_LENGTH];
uint8_t buffer_pointer;
bool cmd_available;

void uart_prints(char input[], const uint8_t length) {
	for (uint8_t i = 0; i < length; i++) {
		Serial.print(input[i]);
	}

	Serial.print('\n');
}

void uart_print_success() {
	Serial.print(0x00);
	Serial.print('\n');
}

void uart_print_error() {
	Serial.print('x');
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
		digitalWrite(DRIVE_LEFT_DIR, LOW);
		analogWrite(DRIVE_LEFT_SPD, buffer[4]);

		uart_print_success();

	// backward
	} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_BACKWARD, CMD_LENGTH) == 0) {
		digitalWrite(DRIVE_LEFT_EN, HIGH);
		digitalWrite(DRIVE_LEFT_DIR, HIGH);
		analogWrite(DRIVE_LEFT_SPD, buffer[4]);

		uart_print_success();

	// stop
	} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_STOP, CMD_LENGTH) == 0) {
		digitalWrite(DRIVE_LEFT_EN, LOW);

		uart_print_success();

	/**
	 * drive engine right side
	 */

	// forward
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_FORWARD, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, HIGH);
		digitalWrite(DRIVE_RIGHT_DIR, HIGH);
		analogWrite(DRIVE_RIGHT_SPD, buffer[4]);

		uart_print_success();

	// backward
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_BACKWARD, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, HIGH);
		digitalWrite(DRIVE_RIGHT_DIR, LOW);
		analogWrite(DRIVE_RIGHT_SPD, buffer[4]);

		uart_print_success();

	// stop
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_STOP, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, LOW);

		uart_print_success();

	/**
	 * rotate engine left side
	 */

	// forward
	} else if (strncmp(cmd, CMD_SET_ROTATE_LEFT_FORWARD, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_LEFT_EN, HIGH);
		digitalWrite(ROTATE_LEFT_DIR, LOW);
		analogWrite(ROTATE_LEFT_SPD, buffer[4]);

		uart_print_success();

	// backward
	} else if (strncmp(cmd, CMD_SET_ROTATE_LEFT_BACKWARD, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_LEFT_EN, HIGH);
		digitalWrite(ROTATE_LEFT_DIR, HIGH);
		analogWrite(ROTATE_LEFT_SPD, buffer[4]);

		uart_print_success();

	// stop
	} else if (strncmp(cmd, CMD_SET_ROTATE_LEFT_STOP, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_LEFT_EN, LOW);

		uart_print_success();

	/**
	 * rotate engine right side
	 */

	// forward
	} else if (strncmp(cmd, CMD_SET_ROTATE_RIGHT_FORWARD, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_RIGHT_EN, HIGH);
		digitalWrite(ROTATE_RIGHT_DIR, LOW);
		analogWrite(ROTATE_RIGHT_SPD, buffer[4]);

		uart_print_success();

	// backward
	} else if (strncmp(cmd, CMD_SET_ROTATE_RIGHT_BACKWARD, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_RIGHT_EN, HIGH);
		digitalWrite(ROTATE_RIGHT_DIR, HIGH);
		analogWrite(ROTATE_RIGHT_SPD, buffer[4]);

		uart_print_success();

	// stop
	} else if (strncmp(cmd, CMD_SET_ROTATE_RIGHT_STOP, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_RIGHT_EN, LOW);

		uart_print_success();

	/**
	 * error
	 */

	// error
	} else {
		uart_print_error();
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

void setup() {

	Serial.begin(BAUDRATE);
	buffer_pointer = 0;
	cmd_available  = false;

	pinMode(DRIVE_LEFT_EN,    OUTPUT);
	pinMode(DRIVE_LEFT_DIR,   OUTPUT);
	pinMode(DRIVE_RIGHT_EN,   OUTPUT);
	pinMode(DRIVE_RIGHT_DIR,  OUTPUT);
	pinMode(ROTATE_LEFT_EN,	  OUTPUT);
	pinMode(ROTATE_LEFT_DIR,  OUTPUT);
	pinMode(ROTATE_RIGHT_EN,  OUTPUT);
	pinMode(ROTATE_RIGHT_DIR, OUTPUT);

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
