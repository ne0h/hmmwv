#include <iostream>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>

#include <joystick.hpp>
#include "gpio.hpp"

using namespace std;

using namespace std;

const int AXIS_MAX = 32767;
const double AXIS_THRS = 0.01;
const double PWM_DUTYMAX = 0.9;
Joystick joystick;
GPIO gpio;

int main() {

	if (!joystick.init()) {
		cout << "Could not find a joystick!\n";
		return 1;
	}

	const string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;

	gpio.setPin(GPIO::P8_08, false);
	gpio.setPwm(GPIO::P8_13, 0.f);

	bool run = true;
	while(run) {
		JoystickEvent event = joystick.getEvent();
		vector<short> axis = event.getAxis();
		vector<bool> buttons = event.getButtons();

		if (buttons.at(0)) {
			run = false;
		}

		double leftController  = (-1.0) * axis.at(1) / AXIS_MAX;
		if (leftController >  PWM_DUTYMAX) leftController =  PWM_DUTYMAX;
		if (leftController < -1.0 * PWM_DUTYMAX) leftController = -1.0 * PWM_DUTYMAX;
		double rightController = (-1.0) * axis.at(2) / AXIS_MAX;
		if (rightController >  PWM_DUTYMAX) rightController =  PWM_DUTYMAX;
		if (rightController < -1.0 * PWM_DUTYMAX) rightController = -1.0 * PWM_DUTYMAX;
		

		if (abs(leftController - AXIS_THRS) && leftController > 0.0) {
			gpio.setPin(GPIO::P8_08, true);
			gpio.setPwm(GPIO::P8_13, abs(leftController));
		} else if (abs(leftController - AXIS_THRS) && leftController < 0.0) {
			gpio.setPin(GPIO::P8_08, false);
			gpio.setPwm(GPIO::P8_13, abs(leftController));
		} else {
			gpio.setPin(GPIO::P8_08, false);
			gpio.setPwm(GPIO::P8_13, 0.0);
		}
	}

	return 0;
}
