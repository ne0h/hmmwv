#include <avr/io.h>
#include <avr/interrupt.h>

#include <usart.hpp>
#include <ssd1306.hpp>
#include <constants.hpp>

#define OLED_LEFT_X         8
#define OLED_RIGHT_X        13
#define OLED_READY          4
#define OLED_STATUS         5
#define OLED_SPEED          6
#define OLED_MONITOR        7  

class Engine {
public:
    uint8_t ready, status, speed, monitor;
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
        break;
    case CMD_RIGHT_STOP:
        break;
    case CMD_LEFT_DRIVE:
        break;
    case CMD_RIGHT_DRIVE:
        break;
    }

    lastCmd = 0;
}

int main() {
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
