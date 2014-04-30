#ifndef GPIO_HPP
#define GPIO_HPP

#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

class GPIO
{
public:
	const unsigned int PWM_PERIOD;

	GPIO();
	~GPIO();
	
	void setPin(int pin, int value);
	void setPwm(const float duty);
	
private:
	bool containsPin(int pin);
	void exportPin(int pin);
	int echo(const char *target, const int value);
	int echo(const char *target, const char *value);

	std::vector<int> _exportedPins;
	std::vector<std::string> _pwmPinPaths;
};

#endif
