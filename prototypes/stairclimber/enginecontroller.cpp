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
#include "i2c.h"

using namespace std;

const double SENSITIVITY	= 16384;
const double PROCESS_NOISE	= 0.1;
const double SENSOR_NOISE	= 64.0;
const double INITIAL_Q		= 100.0;
const int AXIS_MAX			= 32767;

typedef struct {
	double q, r, x, p, k;
} kalman;

Joystick joystick;
kalman kx, ky, kz;

kalman kalman_init(double initial_value) {
	kalman result;

	result.q = PROCESS_NOISE;
	result.r = SENSOR_NOISE;
	result.p = INITIAL_Q;
	result.x = initial_value;

	return result;
}

void kalman_update(kalman *k, double value) {
	k->p = k->p + k->q;
	k->k = k->p / (k->p + k->r);
	k->x = k->x + k->k * (value - k->x);
	k->p = (1 - k->k) * k->p;
}

int main() {
	
	const int addr = 0x68;
	const int fd = i2c_init("/dev/i2c-1");
	
	uint8_t buf[6] = {0};
	i2c_write(fd, addr, buf, 1);
	
	kx = kalman_init(0);
	ky = kalman_init(0);
	kz = kalman_init(0);
	
	while (1) {
		i2c_read(fd, addr, 0x3b, 6, buf);
		const double ax = (uint16_t(((int16_t)buf[0]) << 8) | buf[1]) / SENSITIVITY;
		const double ay = (uint16_t(((int16_t)buf[2]) << 8) | buf[3]) / SENSITIVITY;
		const double az = (uint16_t(((int16_t)buf[4]) << 8) | buf[5]) / SENSITIVITY;
		
		kalman_update(&kx, ax);
		kalman_update(&ky, ay);
		kalman_update(&kz, az);

		double roll  = atan2(-(ky.x), kz.x) * 180.0/M_PI;
		double pitch = atan2(kx.x, sqrt(ky.x*ky.x + kz.x*kz.x)) * 180.0/M_PI;
		
		cout << roll << "\t\t" << pitch << endl;
		
		this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return 0;
}
