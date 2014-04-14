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
	
private:
	bool containsPin(int pin);
	void exportPin(int pin);

	std::vector<int> m_exportedPins;

};

#endif
