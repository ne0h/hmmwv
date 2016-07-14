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
#include "engine.hpp"

using namespace std;

const int AXIS_MAX = 32767;
const double AXIS_THRS = 0.1;
const double PWM_DUTYMAX = 0.99;
Joystick joystick;
GPIO gpio;

int main() {

	if (!joystick.init()) {
		cout << "Could not find a joystick!\n";
		return 1;
	}

	const string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;

	Engine frontLeftEngine (&gpio, GPIO::P8_08, GPIO::P8_13);
	Engine frontRightEngine(&gpio, GPIO::P8_10, GPIO::P8_19);
	Engine backLeftEngine  (&gpio, GPIO::P9_17, GPIO::P9_14);
	Engine backRightEngine (&gpio, GPIO::P9_18, GPIO::P9_16);

	bool run = true;
	while(run) {
		JoystickEvent event = joystick.getEvent();
		vector<short> axis = event.getAxis();
		vector<bool> buttons = event.getButtons();

		if (buttons.at(0)) {
			run = false;
		}

		double accController  = (-1.0) * axis.at(1) / AXIS_MAX;
		if (accController >  PWM_DUTYMAX) accController =  PWM_DUTYMAX;
		if (accController < -1.0 * PWM_DUTYMAX) accController = -1.0 * PWM_DUTYMAX;
		double dirController  =  axis.at(0) / AXIS_MAX;
		if (dirController >  PWM_DUTYMAX) dirController =  PWM_DUTYMAX;
		if (dirController < -1.0 * PWM_DUTYMAX) dirController = -1.0 * PWM_DUTYMAX;

		if (abs(accController) > AXIS_THRS || abs(dirController) > AXIS_THRS) {

			// drive forward
			if (accController >= 0.0) {
				
				const double value = (accController == 0.0) ? 0.99 : 0.99 * accController;
				if (dirController == -0.99) {
					frontLeftEngine.forward(value);
					frontRightEngine.backward(value);
					backLeftEngine.forward(value);
					backRightEngine.backward(value);
					continue;
				} else if (dirController == 0.99) {
					frontLeftEngine.backward(value);
					frontRightEngine.forward(value);
					backLeftEngine.backward(value);
					backRightEngine.forward(value);
					continue;
				}

				if (abs(dirController) < AXIS_THRS) {
					frontLeftEngine.forward(accController);
					frontRightEngine.forward(accController);
					backLeftEngine.forward(accController);
					backRightEngine.forward(accController);
				} else {
					// drive forward left
					if (dirController < 0.0) {
						frontLeftEngine.forward(accController);
						frontRightEngine.backward(1 - abs(accController));
						backLeftEngine.forward(accController);
						backRightEngine.backward(1 - abs(accController));
					// drive forward right
					} else {
						frontLeftEngine.forward(1 - abs(accController));
						frontRightEngine.backward(accController);
						backLeftEngine.forward(1 - abs(accController));
						backRightEngine.backward(accController);
					}
				}
			} else {

				// backwards
				const double value = (accController == 0.0) ? 0.99 : 0.99 * accController;
				if (dirController == -0.99) {
					frontLeftEngine.backward(value);
					frontRightEngine.forward(value);
					backLeftEngine.backward(value);
					backRightEngine.forward(value);
					continue;
				} else if (dirController == 0.99) {
					frontLeftEngine.forward(value);
					frontRightEngine.backward(value);
					backLeftEngine.forward(value);
					backRightEngine.backward(value);
					continue;
				}

				if (abs(dirController) < AXIS_THRS) {
					frontLeftEngine.backward(abs(accController));
					frontRightEngine.backward(abs(accController));
					backLeftEngine.backward(abs(accController));
					backRightEngine.backward(abs(accController));
				} else {
					// drive forward left
					if (dirController < 0.0) {
						frontLeftEngine.backward(accController);
						frontRightEngine.forward(1 - abs(accController));
						backLeftEngine.backward(accController);
						backRightEngine.forward(1 - abs(accController));
					// drive forward right
					} else {
						frontLeftEngine.backward(1 - abs(accController));
						frontRightEngine.forward(accController);
						backLeftEngine.backward(1 - abs(accController));
						backRightEngine.forward(accController);
					}
				}
			}
		} else {
			frontLeftEngine.stop();
			frontRightEngine.stop();
			backLeftEngine.stop();
			backRightEngine.stop();
		}
	}

	return 0;
}
