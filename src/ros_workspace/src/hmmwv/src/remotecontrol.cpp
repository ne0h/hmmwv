#include <vector>
#include <string>

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../include/hmmwv/joystick.hpp"

#define DEAD_ZONE 16384
#define AXIS_MAX  32767

using namespace ros;
using namespace std;

int main(int argc, char **argv) {
	
	// init sdl and connect to controller
	Joystick joystick;
	string name = joystick.getName();
	ROS_INFO("Used controller: %s", name.c_str());
	
	// init ros
	double linear, angular, l_scale, a_scale;

	init(argc, argv, "remotecontrol");
	NodeHandle n;
	Publisher pub;
	
	linear = angular = 0.0;
	l_scale = a_scale = 5.0;
	n.param("scale_angular", a_scale, a_scale);
  	n.param("scale_linear", l_scale, l_scale);

	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	// startup key loop
	// use button.0 to stop
	bool dirty = false;
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
		
		// check if there is an event
		if (abs(axis.at(0)) - DEAD_ZONE > 0
			|| abs(axis.at(1)) - DEAD_ZONE > 0)
			dirty = true;
		
		// calcutelate position values
		angular = (-1.0) * axis.at(0) / AXIS_MAX;	
		linear  = (-1.0) * axis.at(1) / AXIS_MAX;
			
		geometry_msgs::Twist twist;
		twist.angular.z = a_scale * angular;
		twist.linear.x = l_scale  * linear;
		if (dirty == true) {
      		pub.publish(twist);
      		dirty = false;
    		}
	}

	return 0;
}
