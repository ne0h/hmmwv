#include "constants.hpp"
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <limits>

using namespace std;

// A serial interface used to send commands to the Arduino that controls
// the motor controllers. (Yeah, recursion!)
// (C file descriptor)
int tty;
std::stringstream ss;

// for odometry
ros::Time currentTime;
ros::Time lastTime;
ros::Publisher odomPub;
boost::shared_ptr<tf::TransformBroadcaster> odomBroadcaster;
double x = 0;
double y = 0;
double theta = 0;
double vx = 0;
double vy = 0;
double vtheta = 0;
// Make current speeds always available (and query them only once per cycle)
double vLeftCur  = 0.0;
double vRightCur = 0.0;
// Cache speeds to only send data if they change
double vLeftLast     = 0.0;
double vRightLast    = 0.0;
double vRotLeftLast  = 0.0;
double vRotRightLast = 0.0;

// Used to form the Arduino commands
const char MOTOR_LEFT = 'l';
const char MOTOR_RIGHT = 'r';
const char MOTOR_FORWARD = 'f';
const char MOTOR_BACKWARD = 'b';
const char MOTOR_STOP = 's';

// The motors are stopped (not just set to speed 0.0) below this input speed.
const float STOP_THRESHOLD = 0.01f;

bool initTty()
{
	// http://timmurphy.org/2009/08/04/baud-rate-and-other-serial-comm-settings-in-c/
	tty = open("/dev/ttyACM0", O_RDWR | O_NOCTTY/* | O_NDELAY*/);
	if(tty < 0) {
		perror("open(\"/dev/ttyACM0\")");
		return false;
	}
	struct termios settings;
	tcgetattr(tty, &settings);
	cfsetospeed(&settings, B115200);
	cfsetispeed(&settings, B0 /*same as output*/);
	settings.c_cflag &= ~PARENB;	// no parity
	settings.c_cflag &= ~CSTOPB;	// one stop bit
	settings.c_cflag &= ~CSIZE;		// 8 bits per character
	settings.c_cflag |= CS8;
	settings.c_cflag &= ~CRTSCTS;	// Disable hardware flow control
	// settings.c_cflag &= ~HUPCL;		// Send modem disconnect when closing the fd
	settings.c_cflag |= CREAD;		// Enable receiver
	// settings.c_cflag |= CLOCAL;		// Ignore modem status lines
	settings.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable start/stop I/O control
	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Disable user terminal features
	settings.c_oflag &= ~OPOST;		// Disable output postprocessing
	settings.c_cc[VMIN]  = 0;
	settings.c_cc[VTIME] = 0;
	if(tcsetattr(tty, TCSANOW, &settings) != 0) {
		perror("tcsetattr");
		return false;
	}
	tcflush(tty, TCOFLUSH);
	return true;
}

int readLine(char* const line, const ssize_t LINE_LENGTH)
{
	ssize_t bytesRead = 0;
	bool lineComplete = false;
	while(!lineComplete && bytesRead < LINE_LENGTH) {
		// Read bytes into temporary buffer
		char buffer[LINE_LENGTH];
		// ROS_INFO("a");
		ssize_t newBytesRead = read(tty, buffer, LINE_LENGTH);
		// ROS_INFO("b");
		if(newBytesRead < 0) {
			perror("read");
		}
		// Copy new bytes to 'line'
		for(ssize_t i = 0; i < newBytesRead && bytesRead < LINE_LENGTH; i++, bytesRead++) {
			line[bytesRead] = buffer[i];
			if(buffer[i] == '\n') {
				lineComplete = true;
			}
		}
	}
	return bytesRead;
}

void setDrive(const char motor, const char direction, const float spd = 0.0f)
{
	ss.str("");
	ss << "sd" << motor << direction;
	if(direction != MOTOR_STOP) {
		ss << std::hex << (int8_t)(spd * 255);
	}
	ss << std::endl;
	const char* output = ss.str().c_str();
	write(tty, output, ss.str().length());
	// ROS_INFO("%s", output);
	// check response
	char line[2];
	const int lineLength = readLine(line, 2);
	if(atoi(line) != 0) {
		ROS_INFO("Command failed: %s", output);
	}
}

void setRotation(const char motor, const char direction, const float spd = 0.0f)
{
	ss.str("");
	ss << "sr" << motor << direction;
	if(direction != MOTOR_STOP) {
		ss << std::hex << (int8_t)(spd * 255);
	}
	ss << std::endl;
	const char* output = ss.str().c_str();
	write(tty, output, ss.str().length());
	// check response
	char line[2];
	const int lineLength = readLine(line, 2);
	if(atoi(line) != 0) {
		ROS_INFO("Command failed: %s", output);
	}
}

/*
 * Returns the speed in m/s the given motor is moving the robot at.
 */
float getSpeed(const char motor)
{
	// Write command
	ss.str("");
	// "get drive left/right rate"
	ss << "gd" << motor << 'r' << std::endl;
	const char* output = ss.str().c_str();
	write(tty, output, ss.str().length());
	// ROS_INFO("%s", ss.str().c_str());

	// Read response
	// Expected: max. 11 characters (-2,147,483,647) + \n
	const int LINE_LENGTH = 11;
	char line[LINE_LENGTH] = {0};
	const int lineLength = readLine(line, LINE_LENGTH);

	// We receive the interval in µs bewtween two motor monitor ticks
	const int interval = atoi(line);
	// Compute motor revolution freq (Hz) from motor tick interval (µs)
	// Filter out division by zero and "stop" state
	const double freq = (interval == 0 || interval == 1000000) ? 0 : 1000000 / interval;
	// if(motor == MOTOR_RIGHT)
		// ROS_INFO("interval %c: %i freq: %f", motor, interval, freq);
	// Return the resulting robot speed for this motor (m/s)
	return freq * WHEEL_DIAMETER * M_PI / (3.0 * WHEEL_REDUCTION);
}

void velocityCallback(const geometry_msgs::Twist& msg) {
	// Store current motor speeds for later reference
	vLeftCur   = getSpeed(MOTOR_LEFT);
	vRightCur  = getSpeed(MOTOR_RIGHT);

	// Compute the motor speeds necessary to achieve the desired linear and angular motion
	// Adapted from:
	// https://code.google.com/p/differential-drive/source/browse/nodes/twist_to_motors.py
	// self.right = 1.0 * self.dx + self.dr * self.w / 2
	// self.left = 1.0 * self.dx - self.dr * self.w / 2
	double vLeft  = msg.linear.x - msg.angular.z * WHEEL_DISTANCE / (2.0);
	double vRight = msg.linear.x + msg.angular.z * WHEEL_DISTANCE / (2.0);
	double vRotLeft = msg.angular.y;
	double vRotRight = msg.angular.y;

	// Only send new commands to the Arduino if the input actually changed
	if(vLeft == vLeftLast && vRight == vRightLast
		&& vRotLeft == vRotLeftLast && vRotRight == vRotRightLast) {
		return;
	}
	vLeftLast     = vLeft;
	vRightLast    = vRight;
	vRotLeftLast  = vRotLeft;
	vRotRightLast = vRotLeft;

	// ROS_INFO("tl: %f tr: %f z: %f", vLeft, vRight, msg.angular.z);
	// Determine rotation directions
	char dLeft  = vLeft  > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	char dRight = vRight > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;

	// Map [-MAX_DRV_SPEED, MAX_DRV_SPEED] -> [0, 1] as we've extracted the directional component
	vLeft  = vLeft  < 0 ? vLeft  * -1.0 : vLeft;
	vRight = vRight < 0 ? vRight * -1.0 : vRight;
	vLeft  = min(1.0, max(0.0, vLeft  / MAX_DRV_SPEED));
	vRight = min(1.0, max(0.0, vRight / MAX_DRV_SPEED));
	// Stop the motors when stopping
	if(vLeft < STOP_THRESHOLD) {
		dLeft = MOTOR_STOP;
	}
	if(vRight < STOP_THRESHOLD) {
		dRight = MOTOR_STOP;
	}
	// Apply!
	setDrive(MOTOR_LEFT, dLeft, vLeft);
	setDrive(MOTOR_RIGHT, dRight, vRight);

	// Wheel disc rotation
	// Extract direction
	char dRotLeft = vRotLeft > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	char dRotRight = vRotRight > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	// Map speeds to [0, 1]
	vRotLeft  = vRotLeft < 0 ? vRotLeft * -1.0 : vRotLeft;
	vRotRight = vRotRight < 0 ? vRotRight * -1.0 : vRotRight;
	vRotLeft  = min(1.0, max(0.0, vRotLeft  / MAX_ROT_SPEED));
	vRotRight = min(1.0, max(0.0, vRotRight / MAX_ROT_SPEED));
	if(vRotLeft < STOP_THRESHOLD) {
		dRotLeft = MOTOR_STOP;
	}
	if(vRotRight < STOP_THRESHOLD) {
		dRotRight = MOTOR_STOP;
	}
	setRotation(MOTOR_LEFT, dRotLeft, vRotLeft);
	setRotation(MOTOR_RIGHT, dRotRight, vRotRight);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "enginecontrol");
	ros::NodeHandle n;
	currentTime = ros::Time::now();
	lastTime = ros::Time::now();
	if(!initTty()) {
		return 1;
	}

	// Just to make sure we're not going anywhere...
	setDrive(MOTOR_LEFT, MOTOR_STOP);
	setDrive(MOTOR_RIGHT, MOTOR_STOP);
	setRotation(MOTOR_LEFT, MOTOR_STOP);
	setRotation(MOTOR_RIGHT, MOTOR_STOP);

	// This is correct - we're borrowing the turtle's topics
	ros::Subscriber sub = n.subscribe("cmd_vel", 1, velocityCallback);
	ROS_INFO("enginecontrol up and running.");
	ros::spin();
	close(tty);
	return 0;
}
