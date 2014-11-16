#include <avr/io.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE 		9600UL
#define UBRR_VAL 		((F_CPU+BAUDRATE*8)/(BAUDRATE*16)-1)
#define BAUD_REAL 		(F_CPU/(16*(UBRR_VAL+1)))

#ifndef UART_H_
#define UART_H_

void uart_init(void);
void uart_putc(char c);
void uart_puts(char *s);
void uart_puti(uint16_t input);
char uart_getc(void);
void uart_gets(char *buffer, uint8_t maxLength);

#endif
