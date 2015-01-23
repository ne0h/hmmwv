#include "joystick.hpp"
// #include "Rotate.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include <math.h>

const int DEAD_ZONE = 16384;
const int AXIS_MAX = 32767.0;

using namespace ros;
using namespace std;

int main(int argc, char **argv) {
	
	// init sdl and connect to controller
	Joystick joystick;
	string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;
	
	// init ros
	double l_scale = 1.0;
	double a_scale = 1.0;

	init(argc, argv, "remotecontrol");
	NodeHandle n;
	Publisher pub;
	
	n.param("scale_angular", a_scale, a_scale);
  	n.param("scale_linear", l_scale, l_scale);

	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	// Create a fresh publisher before using this line...
	// pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_rotate", 1);

	// startup key loop
	// use button.0 to stop
	bool quit  = false;
	while (!quit) {
		// get axis values
		// use axis.0 for left/right
		// and axis.1 for up/down
		JoystickEvent event = joystick.getEvent();
		vector<short> axis = event.getAxis();
		
		// check for button.0 to stop
		vector<bool> buttons = event.getButtons();
		if (buttons.at(0) == true)
			quit = true;
		
		// calculate position values
		double angular = (-1.0) * axis.at(0) / AXIS_MAX;
		double linear  = (-1.0) * axis.at(1) / AXIS_MAX;
		double stick2y = (-1.0) * axis.at(2) / AXIS_MAX;

		angular = min(max(angular, -1.0), 1.0);
		linear = min(max(linear, -1.0), 1.0);
		stick2y = min(max(stick2y, -1.0), 1.0);
		
		// SPEEED-BUTTON!
		if (!buttons.at(1)) {
			angular *= 0.25;
			linear *= 0.25;
		}
		if (!buttons.at(2)) {
			stick2y *= 0.1; // Scale wheel rotation nice and slow
		}

		if (linear < 0) {
			angular *= (-1.0);
		}

		geometry_msgs::Twist twist;
		twist.angular.z = angular;
		twist.angular.y = stick2y;
		twist.linear.x = linear;

		// Rotate rotate;
		// rotate.linear.x = stick2y;
		
      	pub.publish(twist);
	}

	return 0;
}
