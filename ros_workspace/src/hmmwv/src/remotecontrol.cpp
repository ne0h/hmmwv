#include "joystick.hpp"
#include "constants.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include <math.h>

const int DEAD_ZONE = 16384;
const int AXIS_MAX = 32767;

using namespace ros;
using namespace std;

Joystick joystick;
Publisher pub;

void updateRemote(const TimerEvent&) {
	// get axis values
	// use axis.0 for left/right
	// and axis.1 for up/down
	JoystickEvent event = joystick.getEvent();
	vector<short> axis = event.getAxis();
	vector<bool> buttons = event.getButtons();

	// calculate position values
	double angular = (-1.0) * axis.at(0) / AXIS_MAX;
	double linear  = (-1.0) * axis.at(1) / AXIS_MAX;
	double stick2y = (-1.0) * axis.at(2) / AXIS_MAX;

	angular = min(max(angular, -1.0), 1.0);
	linear = min(max(linear, -1.0), 1.0);
	stick2y = min(max(stick2y, -1.0), 1.0);

	// Normalize driving vector
	// This would require a little more computation in enginecontrol, left out
	// for now.
	/*const double scale = sqrt(pow(angular, 2) + pow(linear, 2));
	angular *= scale;
	linear *= scale;*/

	// SPEEED-BUTTON!
	if (!buttons.at(0)) {
		linear *= 0.25;
	} else {
		// Safety measures...
		angular *= 0.75;
		linear *= 0.75;
	}
	if (!buttons.at(1)) {
		stick2y *= 0.2; // Scale wheel rotation nice and slow
	} else {
		stick2y *= 0.5;
	}

	// if (linear < 0) {
	// 	angular *= (-1.0);
	// }

	// Twist is supposed to contain desired speeds in m/s
	geometry_msgs::Twist twist;
	twist.angular.z = angular * MAX_TURN_SPEED;
	twist.angular.y = stick2y * MAX_ROT_SPEED;
	twist.linear.x = linear * MAX_DRV_SPEED;

	pub.publish(twist);
}

int main(int argc, char **argv) {
	// init sdl and connect to controller
	SDL_Init(SDL_INIT_JOYSTICK);
	if(!joystick.init()) {
		cout << "Could not find a joystick!\n";
		return 1;
	}
	string name = joystick.getName();
	cout << "Used controller: " << name.c_str() << endl;
	
	// init ros
	init(argc, argv, "remotecontrol");
	NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	// startup main loop
	Timer remoteTimer = n.createTimer(Duration(0.0166 /*60 Hz*/), updateRemote);
	spin();
	return 0;
}
