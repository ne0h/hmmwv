#include <util/delay.h>
#include "uart.h"

//4E 99 FF
//CE 99 FF
int main() {

	DDRA  = 0xff;
	PORTA = 0x00;
	uart_init();

	while (1) {
		char c = uart_getc();
		if (c == 0x01) {
			PORTA |= (1 << PA0);
		} else if (c == 0x00) {
			PORTA &= ~(1 << PA0);
		}
	}

	return 0;
}
