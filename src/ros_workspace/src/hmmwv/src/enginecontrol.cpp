#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../include/hmmwv/gpio.hpp"
#include "../include/hmmwv/beaglebone.hpp"

using namespace ros;
using namespace std;

GPIO gpio;

// engine left side

void engine_left_forward(GPIO *gpio) {
	gpio->setPin(ENGINE_DRIVE_LEFT_SPEED, 1);
	gpio->setPin(ENGINE_DRIVE_LEFT_DIRECTION, 1);
	gpio->setPin(ENGINE_DRIVE_LEFT_ENABLE, 1);
}

void engine_left_backward(GPIO *gpio) {
	gpio->setPin(ENGINE_DRIVE_LEFT_SPEED, 1);
	gpio->setPin(ENGINE_DRIVE_LEFT_DIRECTION, 0);
	gpio->setPin(ENGINE_DRIVE_LEFT_ENABLE, 1);
}

void engine_left_stop(GPIO *gpio) {
	gpio->setPin(ENGINE_DRIVE_LEFT_ENABLE, 0);
	gpio->setPin(ENGINE_DRIVE_LEFT_SPEED, 0);
}

// engine right side

void engine_right_forward(GPIO *gpio) {
	gpio->setPin(ENGINE_DRIVE_RIGHT_SPEED, 1);
	gpio->setPin(ENGINE_DRIVE_RIGHT_DIRECTION, 1);
	gpio->setPin(ENGINE_DRIVE_RIGHT_ENABLE, 1);
}

void engine_right_backward(GPIO *gpio) {
	gpio->setPin(ENGINE_DRIVE_RIGHT_SPEED, 1);
	gpio->setPin(ENGINE_DRIVE_RIGHT_DIRECTION, 0);
	gpio->setPin(ENGINE_DRIVE_RIGHT_ENABLE, 1);
}

void engine_right_stop(GPIO *gpio) {
	gpio->setPin(ENGINE_DRIVE_RIGHT_ENABLE, 0);
	gpio->setPin(ENGINE_DRIVE_RIGHT_SPEED, 0);
}

void velocityCallback(const geometry_msgs::Twist& msg) {
	ROS_INFO("linear: %f angular: %f", msg.linear.x, msg.angular.z);
	
	// drive forwards
	if (msg.linear.x > 2.5) {
		engine_left_forward(&gpio);
		engine_right_forward(&gpio);
		
	// drive backward
	} else if (msg.linear.x < -2.5) {
		engine_left_backward(&gpio);
		engine_right_backward(&gpio);
		
	// stop
	} else if (msg.linear.x < 2.5 && msg.linear.x > 2.5) {
		
		// drive left
		if (msg.angular.z > 2.5) {
			engine_left_backward(&gpio);
			engine_right_forward(&gpio);
		} else if (msg.angular.z < -2.5) {
			engine_left_forward(&gpio);
			engine_right_backward(&gpio);
		}
	
		engine_left_stop(&gpio);
		engine_right_stop(&gpio);
	}
}

int main(int argc, char **argv) {

	// init gpio
	GPIO gpio;
	engine_left_stop(&gpio);
	engine_right_stop(&gpio);

	// init ros
	init(argc, argv, "enginecontrol");
	NodeHandle n;
	Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	
	// enter ros loop and wait for callbacks
	spin();

	return 0;
}
