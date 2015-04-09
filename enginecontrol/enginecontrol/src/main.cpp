#include <Arduino.h>

#define BAUDRATE			115200
#define BUFFER_LENGTH 	16

char buffer[BUFFER_LENGTH];
uint8_t buffer_pointer;

void uart_prints(char input[], const uint8_t length) {
	uint8_t i = 0;
	while (i < length) {
		Serial.print(input[i]);
		i++;
	}

	Serial.print('\n');
}

void setup() {
	Serial.begin(BAUDRATE);
	buffer_pointer = 0;
}

void loop() {

	if (Serial.available()) {
		const char c = Serial.read();
		buffer[buffer_pointer] = c;

		if (c == '\n') {
			Serial.print("complete:");
			Serial.print(buffer_pointer);
			Serial.print(" ");
			uart_prints(buffer, buffer_pointer);
			buffer_pointer = 0;
		} else {
			buffer_pointer++;
		}

		// check for buffer overflows
		if (buffer_pointer >= BUFFER_LENGTH) {
			buffer_pointer = 0;
		}
	}

}
