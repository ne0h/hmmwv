#include "pwm.h"

void pwm_init() {
	TCCR1A = (1<<COM1A1) | (1<<WGM11);
	TCCR1B = (1<<WGM13) | (0<<WGM12) | (1<<CS10) |(1<<CS11);

	ICR1  = 1000;
}

void pwm_set(int value) {
	OCR1A = value;
}
