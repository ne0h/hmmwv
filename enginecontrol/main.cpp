#include <avr/io.h>
#include <avr/interrupt.h>

#include <usart.hpp>
#include <ssd1306.hpp>
#include <constants.hpp>
#include <enginecontrol_common.hpp>

#define OLED_LEFT_X             8
#define OLED_RIGHT_X            13
#define OLED_READY              4
#define OLED_STATUS             5
#define OLED_SPEED              6
#define OLED_MONITOR            7

#define ENGINE_LEFT_READY           PC2
#define ENGINE_LEFT_READY_DDR       DDRC
#define ENGINE_LEFT_READY_PIN       PINC
#define ENGINE_LEFT_ENABLE          PD4
#define ENGINE_LEFT_ENABLE_DDR      DDRD
#define ENGINE_LEFT_SPEED           PD5
#define ENGINE_LEFT_SPEED_DDR       DDRD
#define ENGINE_LEFT_DIRECTION       PB4
#define ENGINE_LEFT_DIRECTION_DDR   DDRB

enum EngineId {
    ENGINE_LEFT, ENGINE_RIGHT
};

class Engine {
public:
    uint8_t ready, enable, speed, direction, monitor;
    const uint8_t oledOffset;
    const EngineId engineId;

    Engine(const uint8_t oledOffset, const EngineId engineId) : oledOffset(oledOffset), engineId(engineId) {
        switch (engineId) {
        case ENGINE_LEFT:
            ENGINE_LEFT_ENABLE_DDR    |= (1<<ENGINE_LEFT_ENABLE);
            ENGINE_LEFT_SPEED_DDR     |= (1<<ENGINE_LEFT_SPEED);
            ENGINE_LEFT_DIRECTION_DDR |= (1<<ENGINE_LEFT_DIRECTION);
            break;
        case ENGINE_RIGHT:
            break;
        }
    }
};

static AsyncUSART usart;
SSD1306 oled;
Engine left(OLED_LEFT_X, ENGINE_LEFT);
uint8_t lastCmd = 0;

void update(Engine engine) {
    // read ready status from engine driver
    switch (engine.engineId) {
    case ENGINE_LEFT:
        if (ENGINE_LEFT_READY_PIN & (1<<ENGINE_LEFT_READY)) {
            left.ready = 1;
        }
        break;
    case ENGINE_RIGHT:
        break;
    }

    oled.gotoxy(engine.oledOffset, OLED_READY);
    oled.write("%1i", engine.ready);
    oled.gotoxy(engine.oledOffset, OLED_STATUS);
    oled.write("%1i", engine.enable);
    oled.gotoxy(engine.oledOffset, OLED_SPEED);
    oled.write("%3i", engine.speed);
}

ISR(USART_RX_vect) {
    const uint8_t c = UDR0;

    if (lastCmd == 0) {
        lastCmd = c;
        return;
    }

    oled.gotoxy(0, 2);
    oled.write("0x%2x%2x", lastCmd, c);

    const uint8_t buf[] = {lastCmd, c};
    struct cmd cmd;
    unmarshal(&cmd, buf);

    lastCmd = 0;
}

int main() {
    // enable both channels of Timer0 for PWM
    TCCR0A |= (1<<COM0A1) | (1<<COM0B1);
    TCCR0A |= (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (1<<CS01);

    // draw output structure on oled display
    oled.gotoxy(2, 0);
    oled.write("Enginecontrol");

    oled.gotoxy(OLED_LEFT_X, 2);
    oled.write("L");
    oled.gotoxy(OLED_RIGHT_X, 2);
    oled.write("R");

    oled.gotoxy(0, OLED_READY);
    oled.write("Ready");
    oled.gotoxy(0, OLED_STATUS);
    oled.write("Enable");
    oled.gotoxy(0, OLED_SPEED);
    oled.write("Speed");
    oled.gotoxy(0, OLED_MONITOR);
    oled.write("Monitor");

    update(left);

    sei();

    while (1) {}
	return 0;
}
