#include "uart.h"

void uart_init(void) {
	UBRR0 = UBRR_VAL;
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}

void uart_puts(char *s) {
	while (*s) {
		uart_putc(*s);
		s++;
	}
}

void uart_puti(uint16_t input) {
	char output[16];
	utoa(input, output, 10);
	uart_puts(output);
}

char uart_getc(void) {
	loop_until_bit_is_set(UCSR0A, RXC0);
	return (UDR0);
}

void uart_gets(char *s, uint8_t maxLength) {
	uint8_t length = 0;

	char c = uart_getc();
	while (c != 0x00 && length < maxLength - 1) {
		*s++ = c;

		length++;
		c = uart_getc();
	}

	*s = '\0';
}
