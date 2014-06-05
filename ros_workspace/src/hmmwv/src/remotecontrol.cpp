#include <vector>
#include <string>

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "hmmwv/joystick.hpp"

#define DEAD_ZONE 16384
#define AXIS_MAX  32767

using namespace ros;
using namespace std;

int main(int argc, char **argv) {
	
	// init sdl and connect to controller
	Joystick joystick;
	string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;
	
	// init ros
	double linear, angular, l_scale, a_scale;

	init(argc, argv, "remotecontrol");
	NodeHandle n;
	Publisher pub;
	
	linear = angular = 0.0;
	l_scale = a_scale = 1.0;
	n.param("scale_angular", a_scale, a_scale);
  	n.param("scale_linear", l_scale, l_scale);

	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	// startup key loop
	// use button.0 to stop
	bool quit  = false;
	while (!quit) {
		linear = angular = 0.0;
		
		// get axis values
		// use axis.0 for left/right
		// and axis.1 for up/down
		JoystickEvent event = joystick.getEvent();
		vector<short> axis = event.getAxis();
		
		// check for button.0 to stop
		vector<bool> buttons = event.getButtons();
		if (buttons.at(0) == true)
			quit = true;
		
		// calcutelate position values
		angular = (-1.0) * axis.at(0) / AXIS_MAX;	
		linear  = (-1.0) * axis.at(1) / AXIS_MAX;

		if (angular > 1.0)
			angular = 1.0;

		if (angular < -1.0)
			angular = -1.0;

		if (linear > 1.0)
			linear = 1.0;

		if (linear < -1.0)
			linear = -1.0;
			
		geometry_msgs::Twist twist;
		twist.angular.z = angular;
		twist.linear.x  = linear;
		
      	pub.publish(twist);
	}

	return 0;
}
