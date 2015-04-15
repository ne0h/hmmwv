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
#include <iostream>
#include <sstream>

using namespace std;

// A serial interface used to send commands to the Arduino that controls
// the motor controllers. (Yeah, recursion!)
// (C file descriptor)
int tty;

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
	settings.c_cflag &= ~PARENB;
	settings.c_cflag &= ~CSTOPB;
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8;
	settings.c_cflag &= ~CRTSCTS;
	settings.c_cflag &= ~HUPCL;
	settings.c_cflag |= CREAD | CLOCAL;
	settings.c_iflag &= ~(IXON | IXOFF | IXANY);
	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	settings.c_oflag &= ~OPOST;
	settings.c_cc[VMIN]  = 0;
	settings.c_cc[VTIME] = 0;
	if(tcsetattr(tty, TCSANOW, &settings) != 0) {
		perror("tcsetattr");
		return false;
	}
	tcflush(tty, TCOFLUSH);
	return true;
}

void setDrive(const char motor, const char direction, const float spd = 0.0f)
{
	std::stringstream ss;
	ss << "sd" << motor << direction;
	if(direction != MOTOR_STOP) {
		ss << std::hex << (int8_t)(spd * 255);
	}
	ss << std::endl;
	const char* output = ss.str().c_str();
	write(tty, output, ss.str().length());
	ROS_INFO("%s", output);
}

void setRotation(const char motor, const char direction, const float spd = 0.0f)
{
	std::stringstream ss;
	ss << "sr" << motor << direction;
	if(direction != MOTOR_STOP) {
		ss << std::hex << (int8_t)(spd * 255);
	}
	ss << std::endl;
	const char* output = ss.str().c_str();
	write(tty, output, ss.str().length());
	//ROS_INFO("%s", output);
}

void velocityCallback(const geometry_msgs::Twist& msg) {
	// Store odometry input values
	lastTime = currentTime;
	currentTime = ros::Time::now();
	vx = msg.linear.x;
	vy = msg.linear.y;
	vtheta = msg.angular.z * 10; // absolutely uneducated guess

	// Tank-like steering (rotate around vehicle center)
	// Set linear back/forward speed
	double leftSpd = msg.linear.x;
	double rightSpd = msg.linear.x;
	// Add left/right speeds so that turning on the spot is possible
	leftSpd -= msg.angular.z;
	rightSpd += msg.angular.z;
	// Determine rotation directions
	char leftDir = leftSpd > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	char rightDir = rightSpd > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	// Map [-1, 1] -> [0, 1] as we've extracted the directional component
	leftSpd = leftSpd < 0 ? leftSpd * -1.0 : leftSpd;
	rightSpd = rightSpd < 0 ? rightSpd * -1.0 : rightSpd;
	leftSpd = min(1.0, max(0.0, leftSpd));
	rightSpd = min(1.0, max(0.0, rightSpd));
	// Stop the motors when stopping
	if(leftSpd < STOP_THRESHOLD) {
		leftDir = MOTOR_STOP;
	}
	if(rightSpd < STOP_THRESHOLD) {
		rightDir = MOTOR_STOP;
	}
	// Apply!
	setDrive(MOTOR_LEFT, leftDir, leftSpd);
	setDrive(MOTOR_RIGHT, rightDir, rightSpd);

	// Wheel disc rotation
	double leftRotSpd = msg.angular.y;
	double rightRotSpd = msg.angular.y;
	char leftRotDir = leftRotSpd > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	char rightRotDir = rightRotSpd > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	leftRotSpd = leftRotSpd < 0 ? leftRotSpd * -1.0 : leftRotSpd;
	rightRotSpd = rightRotSpd < 0 ? rightRotSpd * -1.0 : rightRotSpd;
	leftRotSpd = min(1.0, max(0.0, leftRotSpd));
	rightRotSpd = min(1.0, max(0.0, rightRotSpd));
	if(leftRotSpd < STOP_THRESHOLD) {
		leftRotDir = MOTOR_STOP;
	}
	if(rightRotSpd < STOP_THRESHOLD) {
		rightRotDir = MOTOR_STOP;
	}
	setRotation(MOTOR_LEFT, leftRotDir, leftRotSpd);
	setRotation(MOTOR_RIGHT, rightRotDir, rightRotSpd);
}

void publishOdometry(const ros::TimerEvent&) {
	// Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
	// Compute input values
	double dt = (currentTime - lastTime).toSec();
	double dx = (vx * cos(theta) - vy * sin(theta)) * dt;
	double dy = (vx * sin(theta) - vy * cos(theta)) * dt;
	double dtheta = vtheta * dt;
	x += dx;
	y += dy;
	theta += dtheta;

	// Transform frame
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);

	geometry_msgs::TransformStamped odomTrans;
	odomTrans.header.stamp = currentTime;
	odomTrans.header.frame_id = "odom";
	odomTrans.child_frame_id = "base_link";

	odomTrans.transform.translation.x = x;
	odomTrans.transform.translation.y = y;
	odomTrans.transform.translation.z = 0.0;
	odomTrans.transform.rotation = odomQuat;

	odomBroadcaster->sendTransform(odomTrans);

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
	ros::Timer odoTimer = n.createTimer(ros::Duration(1.0/2.0/*2 Hz*/), publishOdometry);
	ROS_INFO("enginecontrol up and running.");
	ros::spin();
	close(tty);
	return 0;
}
