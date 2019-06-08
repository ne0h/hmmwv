#include <avr/io.h>
#include <avr/interrupt.h>

#include <usart.hpp>
#include <ssd1306.hpp>
#include <constants.hpp>

#define OLED_LEFT_X             8
#define OLED_RIGHT_X            13
#define OLED_READY              4
#define OLED_STATUS             5
#define OLED_SPEED              6
#define OLED_MONITOR            7

#define ENGINE_LEFT_READY       PC2
#define ENGINE_LEFT_ENABLE      PD4
#define ENGINE_LEFT_SPEED       PD5
#define ENGINE_LEFT_DIRECTION   PB4

class Engine {
public:
    uint8_t ready, status, speed, direction, monitor;
    const uint8_t x;

    Engine(const uint8_t x) : x(x) {}
};

static AsyncUSART usart;
SSD1306 oled;
Engine left(OLED_LEFT_X), right(OLED_RIGHT_X);
uint16_t lastCmd = 0;

void update(Engine engine) {
    oled.gotoxy(engine.x, OLED_READY);
    oled.write("%1i", engine.ready);
    oled.gotoxy(engine.x, OLED_STATUS);
    oled.write("%1i", engine.status);
    oled.gotoxy(engine.x, OLED_SPEED);
    oled.write("%3i", engine.speed);
}

ISR(USART_RX_vect) {
    const uint8_t c = UDR0;

    if (lastCmd == 0) {
        lastCmd = c << 8;
        return;
    }

    lastCmd += c;
    uint8_t cmd = lastCmd >> 8;
    oled.gotoxy(0, 2);
    oled.write("0x%4x", lastCmd);

    switch (cmd) {
    case CMD_LEFT_STOP:
        left.status = 0;
        left.speed = 0;
        update(left);
        break;
    case CMD_RIGHT_STOP:
        right.status = 0;
        right.speed = 0;
        update(right);
        break;
    case CMD_LEFT_DRIVE_CW:
        left.status = 1;
        left.speed = c;
        left.direction = 0;

        PORTD |= (1<<ENGINE_LEFT_ENABLE);
        PORTB |= (0<<ENGINE_LEFT_DIRECTION);
        OCR0B = c;

        update(left);
        break;
    case CMD_LEFT_DRIVE_CCW:
        left.status = 1;
        left.speed = c;
        left.direction = 1;

        PORTB |= (1<<ENGINE_LEFT_DIRECTION);
        PORTD |= (1<<ENGINE_LEFT_ENABLE);
        OCR0B = c;

        update(left);
        break;
    case CMD_RIGHT_DRIVE_CW:
        right.status = 1;
        right.speed = c;
        update(right);
        break;
    }

    lastCmd = 0;
}

int main() {
    DDRB |= (1<<ENGINE_LEFT_DIRECTION);
    DDRD |= (1<<ENGINE_LEFT_ENABLE) | (1<<ENGINE_LEFT_DIRECTION) | (1<<ENGINE_LEFT_SPEED);

    TCCR0A |= (1<<COM0A1) | (1<<COM0B1);
    TCCR0A |= (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (1<<CS01);

    oled.gotoxy(0, 0);
    if (PINC & (1 << ENGINE_LEFT_READY)) {
        left.ready = 1;
    }

    oled.gotoxy(2, 0);
    oled.write("Enginecontrol");

    oled.gotoxy(OLED_LEFT_X, 2);
    oled.write("L");
    oled.gotoxy(OLED_RIGHT_X, 2);
    oled.write("R");

    oled.gotoxy(0, OLED_READY);
    oled.write("Ready");
    oled.gotoxy(0, OLED_STATUS);
    oled.write("Status");
    oled.gotoxy(0, OLED_SPEED);
    oled.write("Speed");
    oled.gotoxy(0, OLED_MONITOR);
    oled.write("Monitor");

    update(left);
    update(right);

    sei();

    while (1) {}
	return 0;
}
