#ifndef ENGINE_HPP
#define ENGINE_HPP

#include <cmath>
#include <chrono>
#include <thread>
#include <cassert>

#include "gpio.hpp"

class Engine
{
public:
	Engine(GPIO *gpio, const GPIO::Pin directionPin, const GPIO::PwmPin speedPin);
	~Engine();

	/**
	* @brief here be docs
	*/
	void start(const int direction, const double speed = 0.0f);
	void forward(const double speed = 0.0f);
	void backward(const double speed = 0.0f);
	void stop();

private:
	GPIO *_gpio;
	const GPIO::Pin _directionPin;
	const GPIO::PwmPin _speedPin;
	int    _lastDirection;
	double _lastSpeed;
	void rampDownToStop();
};

#endif
