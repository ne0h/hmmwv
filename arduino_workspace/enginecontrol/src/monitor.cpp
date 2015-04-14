#include "monitor.hpp"

void monitor_init(const uint32_t baudrate) {
	Serial2.begin(baudrate);
}

void monitor_write(const char msg[], const uint8_t size) {
	Serial2.print("DEBUG: ");

	for (uint8_t i = 0; i < size; i++) {
		Serial2.print(msg[i]);
	}

	Serial2.print("\n\r");
}
