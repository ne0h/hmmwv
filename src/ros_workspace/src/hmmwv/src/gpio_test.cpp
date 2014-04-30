#include <iostream>
#include "../include/hmmwv/gpio.hpp"

using namespace std;

int main(int argc, char **argv)
{
	GPIO gpio;
	gpio.setPwm(GPIO::P9_16, 1.0);
	cin.get();
	gpio.setPwm(GPIO::P9_16, .5);
	cin.get();
	gpio.setPwm(GPIO::P9_16, 0);
	cin.get();
	return 0;
}
