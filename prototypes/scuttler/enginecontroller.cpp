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

void calculateEngineValues(double accController, double dirController, double *leftValue, double *rightValue) {

	if (abs(accController) > AXIS_THRS || abs(dirController) > AXIS_THRS) {

		// drive forward
		if (accController >= 0.0) {
				
			const double value = (accController == 0.0) ? 0.99 : 0.99 * accController;
			if (dirController == -0.99) {
				*leftValue  = value; //frontLeftEngine.forward(value);
				*rightValue = value * (-1); //frontRightEngine.backward(value);
				return;
			} else if (dirController == 0.99) {
				*leftValue  = value * (-1); //frontLeftEngine.backward(value);
				*rightValue = value; //frontRightEngine.forward(value);
				return;
			}

			if (abs(dirController) < AXIS_THRS) {
				*leftValue  = accController; //frontLeftEngine.forward(accController);
				*rightValue = accController; //frontRightEngine.forward(accController);
			} else {
				// drive forward left
				if (dirController < 0.0) {
					*leftValue  = accController; //frontLeftEngine.forward(accController);
					*rightValue = (-1) * (1 - abs(accController)); //frontRightEngine.backward(1 - abs(accController));
				// drive forward right
				} else {
					*leftValue  = 1 - abs(accController); //frontLeftEngine.forward(1 - abs(accController));
					*rightValue = (-1) * accController; //frontRightEngine.backward(accController);
				}
			}
		} else {

			// backwards
			const double value = (accController == 0.0) ? 0.99 : 0.99 * accController;
			if (dirController == -0.99) {
				*leftValue  = (-1) * value; //frontLeftEngine.backward(value);
				*rightValue = value; //frontRightEngine.forward(value);
				return;
			} else if (dirController == 0.99) {
				*leftValue = value; //frontLeftEngine.forward(value);
				*rightValue = (-1) * value; //frontRightEngine.backward(value);
				return;
			}

			if (abs(dirController) < AXIS_THRS) {
				*leftValue  = (-1) * (abs(accController)); //frontLeftEngine.backward(abs(accController));
				*rightValue = (-1) * (abs(accController)); //frontRightEngine.backward(abs(accController));
			} else {
				// drive forward left
				if (dirController < 0.0) {
					*leftValue  = (-1) * accController; //frontLeftEngine.backward(accController);
					*rightValue = 1 - abs(accController); //frontRightEngine.forward(1 - abs(accController));
				// drive forward right
				} else {
					*leftValue  = (-1) * (1 - abs(accController)); //frontLeftEngine.backward(1 - abs(accController));
					*rightValue = accController; //frontRightEngine.forward(accController);
				}
			}
		}
	} else {
		leftValue = 0; //frontLeftEngine.stop();
		rightValue = 0; //frontRightEngine.stop();
	}
}

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

		double flv, frv;
		calculateEngineValues(accController, dirController, &flv, &frv);

		frontLeftEngine.drive(flv);
		frontRightEngine.drive(frv);
	}

	return 0;
}
