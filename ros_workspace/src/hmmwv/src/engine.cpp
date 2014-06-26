#include <cassert>
#include <stdio.h>
#include "engine.hpp"

Engine::Engine(GPIO *gpio, const GPIO::Pin enablePin, const GPIO::Pin directionPin,
	const GPIO::PwmPin speedPin) :
	_gpio(gpio),
	_enablePin(enablePin),
	_directionPin(directionPin),
	_speedPin(speedPin),
	_lastDirection(0)
{
	gpio->setPin(_enablePin, 0); // Disable first to avoid epileptic motors
	gpio->setPwm(_speedPin, 0.0);
	gpio->setPin(_directionPin, 1);
}

Engine::~Engine() {}

void Engine::setSpeed(const Direction direction, const float speed)
{
	if(speed < 0.0 || speed > 1.0) {
		printf("invalid speed: %f", speed);
		assert(false);
	}

	if(direction != _lastDirection) {
		if(direction == BACKWARD) {
			_gpio->setPin(_enablePin, 1);
			_gpio->setPin(_directionPin, 0);
		}
		else if(direction == FORWARD) {
			_gpio->setPin(_enablePin, 1);
			_gpio->setPin(_directionPin, direction);
		}
		else if(direction == STOP) {
			_gpio->setPin(_enablePin, 0);
			_gpio->setPwm(_speedPin, 0.0);
		}

		_lastDirection = direction;
	}

	_gpio->setPwm(_speedPin, speed);
}
