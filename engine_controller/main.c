#include <util/delay.h>
#include <string.h>

#include "pwm.h"
#include "uart.h"

#define ENGINE_RIGHT_PORT		PORTA
#define ENGINE_RIGHT_ENABLE		PA0
#define ENGINE_RIGHT_DIRECTION	PA1

void engine_right() {
	ENGINE_RIGHT_PORT |= (1 << ENGINE_RIGHT_ENABLE);

	for (int i = 0; i < 1001; i++) {
		pwm_set(i);
		_delay_ms(2);
	}
}

void engine_right_slow_down() {
	for (int i = 1001; i > 0; i--) {
		pwm_set(i);
		_delay_ms(2);
	}
}

void engine_right_stop() {
	ENGINE_RIGHT_PORT &= ~(1 << ENGINE_RIGHT_ENABLE);
}

void engine_right_change_direction() {

}

int main() {
	DDRA = 0x03;
	DDRD = (1 << PD5);

	pwm_init();

	engine_right();
	_delay_ms(10000);
	engine_right_stop();


	while (1) {

	}

	return 0;
}
