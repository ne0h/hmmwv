#pragma once

class Engine {
public:
    uint8_t ready, status, speed, direction, monitor;
    const uint8_t x;

    Engine(const uint8_t x) : x(x) {}
};
