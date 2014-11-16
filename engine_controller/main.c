#include <util/delay.h>
#include <string.h>
#include "uart.h"

#define ENGINE_RIGHT_PORT		PORTA
#define ENGINE_RIGHT_ENABLE		PA0
#define ENGINE_RIGHT_DIRECTION	PA1

//4E 99 FF
//CE 99 FF
int main() {

	DDRA = 0x03;

	uart_init();

	char c;
	while (1) {
		c = uart_getc();

		if (c == 0x00) {
			ENGINE_RIGHT_PORT &= ~(1 << ENGINE_RIGHT_ENABLE);
		} else if (c == 0x01){
			ENGINE_RIGHT_PORT |= (1 << ENGINE_RIGHT_ENABLE);
		} else if (c == 0x02) {
			ENGINE_RIGHT_PORT &= ~(1 << ENGINE_RIGHT_DIRECTION);
		} else if (c == 0x03) {
			ENGINE_RIGHT_PORT |= (1 << ENGINE_RIGHT_DIRECTION);
		}

		/*if (c == 0x01) {
			char task = uart_getc();
			if (task == 0x73) {

				char target = uart_getc();
				int set_value;

				char value  = uart_getc();
				if (value == 0x10) {
					set_value = 0;
				} else {
					set_value = 1;
				}

				if (target == 0x10) {
					ENGINE_RIGHT_PORT |= (set_value << ENGINE_RIGHT_ENABLE);
				}
			}
		}*/
	}

	return 0;
}
