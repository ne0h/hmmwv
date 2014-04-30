#include "../include/hmmwv/gpio.hpp"
#include <cassert>

using namespace std;

GPIO::GPIO() :
	PWM_PERIOD(500000) // nanoseconds = 2000 Hz
{
	// Enable the pwm pins
	echo("/sys/devices/bone_capemgr.9/slots", "am33xx_pwm");
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P8_13");
	//echo("/sys/devices/ocp.3/pwm_test_P8_13.19/period", PWM_PERIOD);
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P8_19");
	//echo("/sys/devices/ocp.3/pwm_test_P8_19.19/period", PWM_PERIOD);
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P9_14");
	//echo("/sys/devices/ocp.3/pwm_test_P9_14.19/period", PWM_PERIOD);
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P9_16");
	echo("/sys/devices/ocp.3/pwm_test_P9_16.19/period", PWM_PERIOD);
}

GPIO::~GPIO()
{

}

void GPIO::setPin(int pin, int value)
{

	// already exported?
	if (!containsPin(pin))
		exportPin(pin);
	
	// set pin
	FILE *outputHandle = NULL;
	char setValue[4], GPIOValue[64];
	
	sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", pin);
	if ((outputHandle = fopen(GPIOValue, "rb+")) == NULL) {
		perror("Failed to open value handle");
		exit(EXIT_FAILURE);
	}
	
	sprintf(setValue, "%d", value);
	fwrite(&setValue, sizeof(char), 1, outputHandle);
	fclose(outputHandle);
}

/* Duty cycle in percent */
void GPIO::setPwm(const float dutyPerc)
{
	/*
	https://groups.google.com/forum/#!topic/beagleboard/qma8bMph0yM

	This will connect PWM to pin P9_14 and generate on the pin  ~2MHz
	waveform with 50% duty.

	modprobe pwm_test
	echo am33xx_pwm > /sys/devices/bone_capemgr.9/slots
	echo bone_pwm_P9_14 > /sys/devices/bone_capemgr.9/slots
	echo 500 > /sys/devices/ocp.2/pwm_test_P9_14.16/period
	echo 250 > /sys/devices/ocp.2/pwm_test_P9_14.16/duty

	Folders in /sys/devices/ocp.3/
	- P9,14: pwm_test_P9_14.16
	- P9,16: pwm_test_P9_16.17
	*/

	const int duty = (1.0 - dutyPerc) * (float)PWM_PERIOD;
	cout << "duty: " << duty << endl;

	int result = 0;
	result = echo("/sys/devices/ocp.3/pwm_test_P9_16.19/duty", duty);

	if(result != 0) {
		cout << "At least one echo failed\n";
	}
}

bool GPIO::containsPin(int pin)
{
	for (vector<int>::iterator it = m_exportedPins.begin();
			it != m_exportedPins.end(); it++) {
		if (*it == pin)
			return true;
	}
	
	return false;
}

void GPIO::exportPin(int pin)
{
	FILE *outputHandle = NULL;
  	char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];
	
	sprintf(GPIOString, "%d", pin);
  	sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", pin);
  	sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", pin);
  	
  	if ((outputHandle = fopen("/sys/class/gpio/export", "ab")) == NULL) {
    	perror("Failed to export pin");
    	exit(EXIT_FAILURE);
  	}

	strcpy(setValue, GPIOString);
	fwrite(&setValue, sizeof(char), 2, outputHandle);
	fclose(outputHandle);
	
	if ((outputHandle = fopen(GPIODirection, "rb+")) == NULL) {
		perror("Failed to open direction handle");
		exit(EXIT_FAILURE);
	}
	
	strcpy(setValue, "out");
	fwrite(&setValue, sizeof(char), 3, outputHandle);
	fclose(outputHandle);
	
	m_exportedPins.push_back(pin);
}

int GPIO::echo(const char *target, const int value)
{
	ofstream file(target);
	if(!file) {
		cerr << "Could not open " << target << endl;
		return -1;
	}

	file << value;
	file.close();
	return 0;
}

int GPIO::echo(const char *target, const char *value)
{
	ofstream file(target);
	if(!file) {
		cerr << "Could not open " << target << endl;
		return -1;
	}

	file << value;
	file.close();
	return 0;
}
