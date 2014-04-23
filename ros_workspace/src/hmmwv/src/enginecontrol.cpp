#include <string>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "gpio.hpp"
#include "engine.hpp"

using namespace ros;
using namespace std;

GPIO gpio;
				
// 						enable		direction		speed
Engine driveLeft(&gpio, GPIO::P9_31, GPIO::P9_21, GPIO::P9_14);
Engine driveRight(&gpio, GPIO::P8_10, GPIO::P8_12, GPIO::P8_13);

void velocityCallback(const geometry_msgs::Twist& msg) {
	//ROS_INFO("%f", msg.linear.x);

	// ================
	// Car-like steering (rotate around inner wheel)
	//double leftSpd = max(0.0, min(1.0, msg.linear.x * (1.0 + msg.angular.x)));
	//double rightSpd = max(0.0, min(1.0, msg.linear.x * (1.0 - msg.angular.x)));
	// ================

	// ================
	// Tank-like steering (rotate around vehicle center)
	// First, set linear back/forward speed
	double leftSpd = msg.linear.x;
	double rightSpd = msg.linear.x;
	// Second, add left/right speeds so that turning on the spot is possible
	leftSpd += msg.angular.x;
	rightSpd -= msg.angular.x;
	// Normalize
	leftSpd = min(0.0, max(1.0, leftSpd));
	rightSpd = min(0.0, max(1.0, rightSpd));
	// ================

	if (msg.linear.x > .1) {
		// drive forward
		ROS_INFO("Forward");
		driveLeft.setSpeed(Engine::BACKWARD, leftSpd);
		driveRight.setSpeed(Engine::FORWARD, rightSpd);
	}
	else if (msg.linear.x < -.1) {
		// drive backward
		ROS_INFO("Backward");
		// The speed value must be in range [0, 1]
		driveLeft.setSpeed(Engine::FORWARD, leftSpd * -1.0);
		driveRight.setSpeed(Engine::BACKWARD, rightSpd * -1.0);
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
