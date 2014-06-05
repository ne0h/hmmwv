#include <cassert>
#include "../include/hmmwv/engine.hpp"

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

void Engine::start(const int direction, const float speed)
{
	assert(direction == -1 || direction == 0 || direction == 1);
	assert(speed >= 0.0 && speed <= 1.0);

	if(direction != _lastDirection) {
		if(direction == -1) {
			_gpio->setPin(_enablePin, 1);
			_gpio->setPin(_directionPin, 0);
		}
		else if(direction == 1) {
			_gpio->setPin(_enablePin, 1);
			_gpio->setPin(_directionPin, direction);
		}

		_lastDirection = direction;
	}

	if(direction != 0) {
		_gpio->setPwm(_speedPin, speed);
	}
	else {
		// Ignore speed settings, we're not supposed to work anyway
		_gpio->setPwm(_speedPin, 0.0);
		_gpio->setPin(_enablePin, 0);
	}
}
