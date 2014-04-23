#include <iostream>
#include "hmmwv/gpio.hpp"

using namespace std;

int main(int argc, char **argv)
{
	GPIO gpio;
	gpio.startPwm(250);
	cin.get();
	return 0;
}
