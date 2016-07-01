#include <Arduino.h>
#include <SPI.h>
#include "constants.hpp"
#include "TFT_22_ILI9225.h"

char buffer[BUFFER_LENGTH];
uint8_t buffer_pointer;
bool cmd_available;

TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED);

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
	char cmd[buffer_pointer];
	memcpy(cmd, buffer, buffer_pointer);

	if (strncmp(cmd, CMD_PRINT_LOAD, CMD_LENGTH) == 0) {
		const uint8_t length = buffer_pointer - CMD_LENGTH;
		String result = String("");
		for (uint8_t i = 0; i < length; i++) {
			result = result + buffer[i+4];
		}

		tft.drawText(10, 10, result, COLOR_GREEN);
	}
	/**
	 * drive engine left side
	 */

	// forward
	/*} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_FORWARD, CMD_LENGTH) == 0) {
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
	/*} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_FORWARD, 4) == 0) {
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
	/*} else if (strncmp(cmd, CMD_SET_ROTATE_LEFT_FORWARD, CMD_LENGTH) == 0) {
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
*/
	/**
	 * rotate engine right side
	 */

	// forward
	/*} else if (strncmp(cmd, CMD_SET_ROTATE_RIGHT_FORWARD, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_RIGHT_EN, HIGH);
		digitalWrite(ROTATE_RIGHT_DIR, HIGH);
		analogWrite(ROTATE_RIGHT_SPD, buffer[4]);

		uart_print_success();

	// backward
	} else if (strncmp(cmd, CMD_SET_ROTATE_RIGHT_BACKWARD, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_RIGHT_EN, HIGH);
		digitalWrite(ROTATE_RIGHT_DIR, LOW);
		analogWrite(ROTATE_RIGHT_SPD, buffer[4]);

		uart_print_success();

	// stop
	} else if (strncmp(cmd, CMD_SET_ROTATE_RIGHT_STOP, CMD_LENGTH) == 0) {
		digitalWrite(ROTATE_RIGHT_EN, LOW);

		uart_print_success();
*/
	/**
	 * error
	 */

	// error
	/*} else {
		uart_print_error();
	}*/

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
	tft.begin();
	tft.setFont(Terminal12x16);
	tft.setBacklight(HIGH);
	tft.setOrientation(1);

	Serial.begin(BAUDRATE);
}

void loop() {
	if (cmd_available) {
		cmd_available = false;
		cmd();
	}
}
