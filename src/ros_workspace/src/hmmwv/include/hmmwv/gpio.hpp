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
	enum PIN {
		P8_13 = 0,
		P8_19 = 1,
		P9_14 = 2,
		P9_16 = 3
	};

	GPIO();
	~GPIO();
	
	void setPin(int pin, int value);
	void setPwm(const PIN pin, const float duty);
	
private:
	bool containsPin(int pin);
	void exportPin(int pin);
	int echo(const std::string target, const int value);
	int echo(const std::string target, const char *value);
	std::string matchPath(std::string pattern);
	inline std::string append(const std::string base, const std::string suffix)
	{
		std::string tmp(base);
		tmp.append(suffix);
		return tmp;
	}

	const unsigned int PWM_PERIOD;
	std::vector<int> _exportedPins;
	std::vector<std::string> _pwmPinPaths;
};

#endif
