#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../include/hmmwv/gpio.hpp"
#include "../include/hmmwv/engine.hpp"

using namespace ros;
using namespace std;

GPIO gpio;
						// enable	direction		speed
Engine driveLeft(&gpio, GPIO::P8_10, GPIO::P8_12, GPIO::P8_13);
Engine driveRight(&gpio, GPIO::P9_21, GPIO::P9_22, GPIO::P9_14);
//Engine rotatorLeft(..);
//Engine rotatorRight(..);

void velocityCallback(const geometry_msgs::Twist& msg) {
	ROS_INFO("%f", msg.linear.x);

	if (msg.linear.x > .1) {
		// drive forward
		driveLeft.start(-1, msg.linear.x);
		driveRight.start(1, msg.linear.x);
	}
	else if (msg.linear.x < -.1) {
		// drive backward
		driveLeft.start(1, msg.linear.x);
		driveRight.start(-1, msg.linear.x);
	}
	else {
		// stop
		driveLeft.start(0);
		driveRight.start(0);
	}
}

int main(int argc, char **argv) {
	driveLeft.start(0);
	driveRight.start(0);

	// init ros
	init(argc, argv, "enginecontrol");
	NodeHandle n;
	Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	ROS_INFO("enginecontrol up and running.");
	
	// enter ros loop and wait for callbacks
	spin();
	return 0;
}
