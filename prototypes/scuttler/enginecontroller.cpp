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
				*leftValue  = value;
				*rightValue = value * (-1);
				return;
			} else if (dirController == 0.99) {
				*leftValue  = value * (-1);
				*rightValue = value;
				return;
			}

			if (abs(dirController) < AXIS_THRS) {
				*leftValue  = accController;
				*rightValue = accController;
			} else {
				// drive forward left
				if (dirController < 0.0) {
					*leftValue  = accController;
					*rightValue = (-1) * (1 - abs(accController));
				// drive forward right
				} else {
					*leftValue  = 1 - abs(accController);
					*rightValue = (-1) * accController;
				}
			}
		} else {

			// backwards
			const double value = (accController == 0.0) ? 0.99 : 0.99 * accController;
			if (dirController == -0.99) {
				*leftValue  = (-1) * value;
				*rightValue = value;
				return;
			} else if (dirController == 0.99) {
				*leftValue = value;
				*rightValue = (-1) * value;
				return;
			}

			if (abs(dirController) < AXIS_THRS) {
				*leftValue  = (-1) * (abs(accController));
				*rightValue = (-1) * (abs(accController));
			} else {
				// drive forward left
				if (dirController < 0.0) {
					*leftValue  = (-1) * accController;
					*rightValue = 1 - abs(accController);
				// drive forward right
				} else {
					*leftValue  = (-1) * (1 - abs(accController));
					*rightValue = accController;
				}
			}
		}
	} else {
		*leftValue = 0;
		*rightValue = 0;
	}
}

int main() {

	if (!joystick.init()) {
		cout << "Could not find a joystick!\n";
		return 1;
	}

	const string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;
	
	//							   direction    speed
	Engine frontLeftEngine (&gpio, GPIO::P9_23, GPIO::P9_14);
	Engine frontRightEngine(&gpio, GPIO::P9_24, GPIO::P9_16);
	Engine backLeftEngine  (&gpio, GPIO::P8_08, GPIO::P8_13);
	Engine backRightEngine (&gpio, GPIO::P8_10, GPIO::P8_19);
	
	frontLeftEngine.stop();
	frontRightEngine.stop();
	backLeftEngine.stop();
	backRightEngine.stop();

	bool run = true;
	while(run) {
		JoystickEvent event = joystick.getEvent();
		vector<short> axis = event.getAxis();
		vector<bool> buttons = event.getButtons();

		if (buttons.at(0)) {
			run = false;
		}

		/**
		 * front engines
		 */

		double frontAccController  = (-1.0) * axis.at(1) / AXIS_MAX;
		if (frontAccController >  PWM_DUTYMAX)       frontAccController =  PWM_DUTYMAX;
		if (frontAccController < -1.0 * PWM_DUTYMAX) frontAccController = -1.0 * PWM_DUTYMAX;
		double frontDirController  =  axis.at(0) / AXIS_MAX;
		if (frontDirController >  PWM_DUTYMAX)       frontDirController =  PWM_DUTYMAX;
		if (frontDirController < -1.0 * PWM_DUTYMAX) frontDirController = -1.0 * PWM_DUTYMAX;

		double fflv, ffrv;
		calculateEngineValues(frontAccController, frontDirController, &fflv, &ffrv);
		frontLeftEngine.drive(fflv);
		frontRightEngine.drive(ffrv);
		
		/**
		 * back engines
		 */
		 
		double backAccController  = (-1.0) * axis.at(3) / AXIS_MAX;
		if (backAccController >  PWM_DUTYMAX)       backAccController =  PWM_DUTYMAX;
		if (backAccController < -1.0 * PWM_DUTYMAX) backAccController = -1.0 * PWM_DUTYMAX;
		double backDirController  =  axis.at(2) / AXIS_MAX;
		if (backDirController >  PWM_DUTYMAX)       backDirController =  PWM_DUTYMAX;
		if (backDirController < -1.0 * PWM_DUTYMAX) backDirController = -1.0 * PWM_DUTYMAX;

		double bflv, bfrv;
		calculateEngineValues(backAccController, backDirController, &bflv, &bfrv);
		backLeftEngine.drive(bflv);
		backRightEngine.drive(bfrv);
	}

	return 0;
}
