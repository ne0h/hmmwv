#include "gpio.hpp"
#include "engine.hpp"

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
		driveLeft.setDirection(Engine::BACKWARD);
		driveLeft.setSpeed(leftSpd);
		driveRight.setDirection(Engine::FORWARD);
		driveRight.setSpeed(rightSpd);
	}
	else if (msg.linear.x < -.1) {
		// drive backward
		ROS_INFO("Backward");
		driveLeft.setDirection(Engine::FORWARD);
		driveLeft.setSpeed(leftSpd * -1.0);
		driveRight.setDirection(Engine::BACKWARD);
		driveRight.setSpeed(rightSpd * -1.0);
	}
	else {
		// stop
		driveLeft.setDirection(Engine::STOP);
		driveRight.setDirection(Engine::STOP);
	}
}

int main(int argc, char **argv) {
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
