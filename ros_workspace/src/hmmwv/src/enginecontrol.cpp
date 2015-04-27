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
// // direction cache (computed in velocityCallback, used for odometry)
int dLeftOdo = 1;
int dRightOdo = 1;

const double WHEEL_DIAMETER		= 0.155;
const double WHEEL_OFFSET		= 0.9;//0.215;
const double WHEEL_REDUCTION	= 86.0;

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
	// write(tty, output, ss.str().length());
	// check response
	// char line[2];
	// const int lineLength = readLine(line, 2);
	// if(atoi(line) != 0) {
	// 	ROS_INFO("Command failed: %s", output);
	// }
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
	// Expected: max. 10 characters (2,147,483,647) + \n
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
	// Tank-like steering (rotate around vehicle center)
	// Set linear back/forward speed
	double vLeft = msg.linear.x;
	double vRight = msg.linear.x;
	// Add left/right speeds so that turning on the spot is possible
	vLeft -= msg.angular.z;
	vRight += msg.angular.z;
	// Determine rotation directions
	// (Yes, having two kinds of "direction" variables sucks badly. Another take
	// at object orientation should be done to fix this.)
	char dLeft = vLeft > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	char dRight = vRight > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	dLeftOdo = vLeft > 0 ? 1 : -1;
	dRightOdo = vRight > 0 ? 1 : -1;
	// Map [-1, 1] -> [0, 1] as we've extracted the directional component
	vLeft = vLeft < 0 ? vLeft * -1.0 : vLeft;
	vRight = vRight < 0 ? vRight * -1.0 : vRight;
	vLeft = min(1.0, max(0.0, vLeft));
	vRight = min(1.0, max(0.0, vRight));
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
	double vRotLeft = msg.angular.y;
	double vRotRight = msg.angular.y;
	char dRotLeft = vRotLeft > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	char dRotRight = vRotRight > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	vRotLeft = vRotLeft < 0 ? vRotLeft * -1.0 : vRotLeft;
	vRotRight = vRotRight < 0 ? vRotRight * -1.0 : vRotRight;
	vRotLeft = min(1.0, max(0.0, vRotLeft));
	vRotRight = min(1.0, max(0.0, vRotRight));
	if(vRotLeft < STOP_THRESHOLD) {
		dRotLeft = MOTOR_STOP;
	}
	if(vRotRight < STOP_THRESHOLD) {
		dRotRight = MOTOR_STOP;
	}
	setRotation(MOTOR_LEFT, dRotLeft, vRotLeft);
	setRotation(MOTOR_RIGHT, dRotRight, vRotRight);
}

void publishOdometry(const ros::TimerEvent&) {
// 	// Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
// 	// Store odometry input values
	double vLeft = getSpeed(MOTOR_LEFT) * dLeftOdo;
	double vRight = getSpeed(MOTOR_RIGHT) * dRightOdo;
	vx = (vLeft + vRight) / 2.0;
	// vtheta = (vRight - vLeft) / WHEEL_OFFSET;

	// Compute input values
	double dt = (currentTime - lastTime).toSec();
	lastTime = currentTime;
	currentTime = ros::Time::now();
	double dx = (vx * cos(theta) - vy * sin(theta)) * dt;
	double dy = (vx * sin(theta) - vy * cos(theta)) * dt;
	vtheta = (dx - dy) / WHEEL_OFFSET;
	double dtheta = vtheta * dt;
	x += dx;
	y += dy;
	theta += dtheta;
	// ROS_INFO("vl: %f vr: %f vx: %f vtheta: %f", vLeft, vRight, vx, vtheta);

	// Transform frame
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);

	// geometry_msgs::TransformStamped odomTrans;
	// odomTrans.header.stamp = currentTime;
	// odomTrans.header.frame_id = "odom";
	// odomTrans.child_frame_id = "base_link";

	// odomTrans.transform.translation.x = x;
	// odomTrans.transform.translation.y = y;
	// odomTrans.transform.translation.z = 0.0;
	// odomTrans.transform.rotation = odomQuat;

	// odomBroadcaster->sendTransform(odomTrans);


	// Odometry message
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odomQuat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vtheta;

	odomPub.publish(odom);

	// Test TF
	// <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser" args="0.1 0.1 0.15 0.0 0.0 0.0 /base_link /laser 50" />
	// geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0.0);
	// geometry_msgs::TransformStamped odomTrans;
	// odomTrans.header.stamp = ros::Time::now();
	// odomTrans.header.frame_id = "base_link";
	// odomTrans.child_frame_id = "laser";
	// odomTrans.transform.translation.x = 0.1;
	// odomTrans.transform.translation.y = 0.1;
	// odomTrans.transform.translation.z = 0.15;
	// odomTrans.transform.rotation = odomQuat;
	// odomBroadcaster->sendTransform(odomTrans);
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
	ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	odomPub = n.advertise<nav_msgs::Odometry>("odom", 50);
	odomBroadcaster = boost::make_shared<tf::TransformBroadcaster>();
	ros::Timer odoTimer = n.createTimer(ros::Duration(1.0/10.0/*10 Hz*/), publishOdometry);
	ROS_INFO("enginecontrol up and running.");
	ros::spin();
	close(tty);
	return 0;
}
