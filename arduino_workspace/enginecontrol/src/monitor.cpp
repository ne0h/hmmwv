#include "monitor.hpp"

void monitor_init(const uint32_t baudrate) {
	Serial3.begin(baudrate);
}

void monitor_write(const char msg[], const uint8_t size) {
	Serial3.print("DEBUG: ");

	for (uint8_t i = 0; i < size; i++) {
		Serial3.print(msg[i]);
	}

	Serial3.print("\n\r");
}
