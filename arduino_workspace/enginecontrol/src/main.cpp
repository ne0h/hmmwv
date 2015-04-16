#include <Arduino.h>

#include "constants.h"
#include "monitor.hpp"

char buffer[BUFFER_LENGTH];
uint8_t buffer_pointer;
bool cmd_available;

// The motor monitor ISR counts ticks into this variable
volatile uint8_t leftCurrentTicks = 0;
volatile uint8_t rightCurrentTicks = 0;
// Another timed ISR regularly copies the value of currentTicks into this
// variable to make it available for reading via serial.
volatile uint8_t leftAggregatedTicks = 0;
volatile uint8_t rightAggregatedTicks = 0;

void uart_prints(char input[], const uint8_t length) {
	for (uint8_t i = 0; i < length; i++) {
		Serial.print(input[i]);
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
		digitalWrite(DRIVE_LEFT_DIR, LOW);
		analogWrite(DRIVE_LEFT_SPD, buffer[4]);

	// backward
	} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_BACKWARD, CMD_LENGTH) == 0) {
		digitalWrite(DRIVE_LEFT_EN, HIGH);
		digitalWrite(DRIVE_LEFT_DIR, HIGH);
		analogWrite(DRIVE_LEFT_SPD, buffer[4]);

	// stop
	} else if (strncmp(cmd, CMD_SET_DRIVE_LEFT_STOP, CMD_LENGTH) == 0) {
		digitalWrite(DRIVE_LEFT_EN, LOW);

	/**
	 * drive engine right side
	 */

	// forward
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_FORWARD, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, HIGH);
		digitalWrite(DRIVE_RIGHT_DIR, HIGH);
		analogWrite(DRIVE_RIGHT_SPD, buffer[4]);

	// backward
	} else if (strncmp(cmd, CMD_SET_DRIVE_RIGHT_BACKWARD, 4) == 0) {
		digitalWrite(DRIVE_RIGHT_EN, HIGH);
		digitalWrite(DRIVE_RIGHT_DIR, LOW);
		analogWrite(DRIVE_RIGHT_SPD, buffer[4]);

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

/*
 * Interrupt Service Routine for the left drive motor monitor.
 * Careful, this might be called at up to 500 Hz.
 */
void isr_drive_left_mnt()
{
	/*
	 * Inside the attached function, delay() won't work and the value returned
	 * by millis() will not increment. Serial data received while in the
	 * function may be lost. You should declare as volatile any variables that
	 * you modify within the attached function.
	 */
	leftCurrentTicks++;
}

/*
 * Interrupt Service Routine for the right drive motor monitor.
 * Careful, this might be called at up to 500 Hz.
 */
void isr_drive_right_mnt()
{
	rightCurrentTicks++;
}

/*
 * ISR that makes current motor tick counts available for reading.
 */
void isr_drive_mnt_reset()
{
	leftAggregatedTicks = leftCurrentTicks;
	leftCurrentTicks = 0;
	rightAggregatedTicks = rightCurrentTicks;
	rightCurrentTicks = 0;
}

void setup() {
	monitor_init(115200);

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

	monitor_write("pin modes set", 13);

	// setup motor encoder listeners
	attachInterrupt(DRIVE_LEFT_MNT, &isr_drive_left_mnt, RISING);
	attachInterrupt(DRIVE_RIGHT_MNT, &isr_drive_right_mnt, RISING);
	// If we had Timer3 available, we could do
	//Timer3.attachInterrupt(&isr_drive_mnt_reset, 100000 /*µs = 10 Hz*/);
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
