#ifndef SRC_MONITOR_HPP_
#define SRC_MONITOR_HPP_

#include <Arduino.h>

void monitor_init(const uint32_t baudrate);
void monitor_write(const char *msg, const uint8_t size);

#endif /* SRC_MONITOR_HPP_ */
