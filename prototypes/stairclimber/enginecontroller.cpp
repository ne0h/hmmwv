#include <iostream>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <joystick.hpp>

using namespace std;

const int AXIS_MAX = 32767;
Joystick joystick;

int writeSysFs(const string path, const string value) {
	ofstream file(path.c_str());
	if (!file) {
		cerr << "Could not open " << path << endl;
		return -1;
	}

	file << value;
	file.close();

	return 0;
}

int writeSysFs(const string path, const int value) {
	writeSysFs(path, to_string(value));
}

void exportPin(const int pin) {
	writeSysFs("/sys/class/gpio/export", pin);

	stringstream ss;
	ss << "/sys/class/gpio/gpio" << pin << "/direction";
	writeSysFs(ss.str(), "out");
}

void setPin(const int pin, const int value) {
	stringstream ss;
	ss << "/sys/class/gpio/gpio" << pin << "/value";
	writeSysFs(ss.str(), value);
}

int main() {

	if (!joystick.init()) {
		cout << "Could not find a joystick!\n";
		return 1;
	}

	const string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;

	exportPin(67);
	exportPin(68);

	bool run = true;
	while(run) {
		JoystickEvent event = joystick.getEvent();
		vector<short> axis = event.getAxis();
		vector<bool> buttons = event.getButtons();

		if (buttons.at(0)) {
			run = false;
		}

		double leftController  = (-1.0) * axis.at(1) / AXIS_MAX;
		double rightController = (-1.0) * axis.at(2) / AXIS_MAX;

		if (leftController > 0.0f) {
			setPin(67, 1);
		} else {
			setPin(67, 0);
		}
	}

	return 0;
}
