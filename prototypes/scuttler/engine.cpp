#include "engine.hpp"
#include <iostream>

Engine::Engine(GPIO *gpio, const GPIO::Pin directionPin, const GPIO::PwmPin speedPin) :
	_gpio(gpio),
	_directionPin(directionPin),
	_speedPin(speedPin),
	_lastDirection(0)
{
	_lastSpeed = 0.0;
	_lastDirection = 0;

	gpio->setPwm(_speedPin, 0.0);
	gpio->setPin(_directionPin, 1);
}

Engine::~Engine() {}

void Engine::start(const int direction, const double speed)
{
	assert(direction == -1 || direction == 0 || direction == 1);
	if(!(speed >= 0.0 && speed <= 1.0)) {
		printf("invalid speed: %f", speed);
		assert(false);
	}

	_gpio->setPin(_directionPin, direction);
	_gpio->setPwm(_speedPin, speed);

	_lastDirection = direction;
	_lastSpeed = speed;
}

void Engine::forward(const double speed) {
	start(0, speed);
}

void Engine::backward(const double speed) {
	start(1, speed);
}

void Engine::stop() {
	start(0, 0.0);
	//rampDownToStop();
}

void Engine::rampDownToStop() {
	std::cout << floor(_lastSpeed * 10) << std::endl;
	for (unsigned int i = floor(_lastSpeed * 10); i > 0; i--) {
		start(_lastDirection, i / 10.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	stop();
}

void Engine::drive(const double value) {
	if (value > 0) {
		forward(value);
	} else if (value < 0) {
		backward(fabs(value));
	} else {
		stop();
	}
}
