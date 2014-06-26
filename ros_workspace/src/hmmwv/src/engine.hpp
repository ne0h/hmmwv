#ifndef ENGINE_HPP
#define ENGINE_HPP

#include "gpio.hpp"

class Engine
{
public:
	/**
	 * Describes the *motor's* rotation direction, not the
	 * robot's movement direction!
	 */
	enum Direction {
		FORWARD,
		STOP,
		BACKWARD
	}

	/**
	 * @brief Creates a new engine instance. Pins are expected to be the ones that
	 * are connected to the respective motor pins. The speed pin must be a PWM pin.
	 * 
	 * @param gpio Pointer to the GPIO instance that controls the other pins
	 * @param enablePin The "enable" pin.
	 * @param directionPin The "direction" pin.
	 * @param speedPin The "speed" PWM pin.
	 */
	Engine(GPIO *gpio, const GPIO::Pin enablePin, const GPIO::Pin directionPin,
		const GPIO::PwmPin speedPin);
	~Engine();

	/**
	 * @brief Set the motor direction and speed.
	 * 
	 * @param direction Motor direction or Direction::STOP. When direction is
	 * STOP, the speed value is forced to 0.0f and the "enable" pin is switched off.
	 * @param speed Defines the duty cycle percent for speed PWM, 0 <= speed <= 1.
	 */
	void setSpeed(const Direction direction, const float speed = 0.0f);

private:
	GPIO *_gpio;
	const GPIO::Pin _enablePin;
	const GPIO::Pin _directionPin;
	const GPIO::PwmPin _speedPin;
	int _lastDirection;
};

#endif
