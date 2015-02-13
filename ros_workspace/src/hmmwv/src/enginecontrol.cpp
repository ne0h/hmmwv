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
Engine rotatorLeft(&gpio, GPIO::P9_24, GPIO::P9_26, GPIO::P9_16);
Engine rotatorRight(&gpio, GPIO::P8_17, GPIO::P8_15, GPIO::P8_19);

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
	// if(msg.linear.x >= 0) {
		leftSpd -= msg.angular.z;
		rightSpd += msg.angular.z;
	// } else {
	// 	leftSpd += msg.angular.z;
	// 	rightSpd -= msg.angular.z;
	// }
	// Determine rotation directions
	Engine::Direction leftDir = leftSpd > 0 ? Engine::BACKWARD : Engine::FORWARD;
	Engine::Direction rightDir = rightSpd > 0 ? Engine::FORWARD : Engine::BACKWARD;
	// Normalize
	leftSpd = leftSpd < 0 ? leftSpd * -1.0 : leftSpd;
	rightSpd = rightSpd < 0 ? rightSpd * -1.0 : rightSpd;
	leftSpd = min(1.0, max(0.0, leftSpd));
	rightSpd = min(1.0, max(0.0, rightSpd));
	// Apply!
	driveLeft.setDirection(leftDir);
	driveLeft.setSpeed(leftSpd);
	driveRight.setDirection(rightDir);
	driveRight.setSpeed(rightSpd);
	// ================

	// Wheel disc rotation
	double leftRotSpd = msg.angular.y;
	double rightRotSpd = msg.angular.y;
	Engine::Direction leftRotDir = leftRotSpd > 0 ? Engine::BACKWARD : Engine::FORWARD;
	Engine::Direction rightRotDir = rightRotSpd > 0 ? Engine::FORWARD : Engine::BACKWARD;
	leftRotSpd = leftRotSpd < 0 ? leftRotSpd * -1.0 : leftRotSpd;
	rightRotSpd = rightRotSpd < 0 ? rightRotSpd * -1.0 : rightRotSpd;
	leftRotSpd = min(1.0, max(0.0, leftRotSpd));
	rightRotSpd = min(1.0, max(0.0, rightRotSpd));
	rotatorLeft.setDirection(leftRotDir);
	rotatorLeft.setSpeed(leftRotSpd);
	rotatorRight.setDirection(rightRotDir);
	rotatorRight.setSpeed(leftRotSpd);
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
