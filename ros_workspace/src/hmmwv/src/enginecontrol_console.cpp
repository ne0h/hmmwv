#include <iostream>
#include <cmath>

#include "console.hpp"
#include "constants.hpp"
#include "enginecontrol_common.hpp"
#include <gamepad.hpp>

Console console("/dev/cu.wchusbserial1420");

void cb(gamepad::Event &event) {
    auto axis = event.getAxis();

    uint8_t speed = (uint8_t)(abs(axis[1]) * 255 / 32768);
    if (abs(axis[1]) < 5000) {
        speed = 0;
    }

    uint8_t direction;
    if (axis[1] > 0) {
        direction = CCW;
    } else {
        direction = CW;
    }
    struct cmd cmd = {
        1, ENGINE_LEFT, direction, speed
    };

    uint8_t buf[2];
    marshal(&cmd, buf);
    console.send(buf, 2);
}

int main(int argc, char *argv[]) {
    gamepad::Gamepad gamepad;
    gamepad.addCallback(cb);

    std::cin.ignore();
    console.send((uint16_t)0);
    return 0;
}
