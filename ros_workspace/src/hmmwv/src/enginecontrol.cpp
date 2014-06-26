#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "gpio.hpp"
#include "engine.hpp"

using namespace ros;
using namespace std;

GPIO gpio;
						// enable	direction		speed
Engine driveLeft(&gpio, GPIO::P8_10, GPIO::P8_12, GPIO::P8_13);
Engine driveRight(&gpio, GPIO::P9_23, GPIO::P9_21, GPIO::P9_14);
//Engine rotatorLeft(..);
//Engine rotatorRight(..);

void velocityCallback(const geometry_msgs::Twist& msg) {
	//ROS_INFO("%f", msg.linear.x);

	if (msg.linear.x > .1) {
		// drive forward
		ROS_INFO("Forward");
		driveLeft.setSpeed(Engine::BACKWARD, msg.linear.x);
		driveRight.setSpeed(Engine::FORWARD, msg.linear.x);
	}
	else if (msg.linear.x < -.1) {
		// drive backward
		ROS_INFO("Backward");
		// The speed value must be in range [0, 1]
		driveLeft.setSpeed(Engine::FORWARD, msg.linear.x * -1.0);
		driveRight.setSpeed(Engine::BACKWARD, msg.linear.x * -1.0);
	}
	else {
		// stop
		driveLeft.setSpeed(Engine::STOP);
		driveRight.setSpeed(Engine::STOP);
	}
}

int main(int argc, char **argv) {
	//driveLeft.start(0); // Why were we doing that? oO
	//driveRight.start(0);

	// Set reasonable defaults for not actively used pins
	//gpio.setPin(GPIO::P8_8, false);

	// init ros
	init(argc, argv, "enginecontrol");
	NodeHandle n;
	// This is correct - we're borrowing the turtle's topics
	Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	ROS_INFO("enginecontrol up and running.");
	
	// enter ros loop and wait for callbacks
	spin();
	return 0;
}
