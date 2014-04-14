#include "gpio.hpp"

using namespace std;

GPIO::GPIO() {
	vector<int> m_exportedPins;
}

GPIO::~GPIO() {

}

void GPIO::setPin(int pin, int value) {

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

bool GPIO::containsPin(int pin) {
	for (vector<int>::iterator it = m_exportedPins.begin();
			it != m_exportedPins.end(); it++) {
		if (*it == pin)
			return true;
	}
	
	return false;
}

void GPIO::exportPin(int pin) {
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
