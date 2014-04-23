#ifndef GPIO_HPP
#define GPIO_HPP

#include <cstdlib>
#include <vector>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

class GPIO {
public:
	GPIO();
	~GPIO();
	
	void setPin(int pin, int value);
	void startPwm(const int duty);
	
private:
	bool containsPin(int pin);
	void exportPin(int pin);
	int echo(const char *target, const unsigned short value);
	int echo(const char *target, const char *value);

	std::vector<int> m_exportedPins;

};

#endif
