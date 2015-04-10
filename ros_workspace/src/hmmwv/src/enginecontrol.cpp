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
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

// A serial interface used to send commands to the Arduino that controls
// the motor controllers. (Yeah, recursion!)
//std::fstream tty;
FILE* tty;

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

bool initTty()
{
	// http://timmurphy.org/2009/08/04/baud-rate-and-other-serial-comm-settings-in-c/
	//tty.open("/dev/ttyACM0", std::fstream::in | std::fstream::out | std::fstream::app);
	/*if(!tty.is_open()) {
		ROS_INFO("Couldn't open /dev/ttyACM0. Is the Arduino plugged in?");
		std::cerr << "Couldn't open /dev/ttyACM0. Is the Arduino plugged in?\n" << std::flush;
		ros::shutdown();
		return false;
	}
	tty << std::hex;*/

	int ttyFd = open("/dev/ttyACM0", O_RDWR);
	if(ttyFd < 0) {
		ROS_INFO("Couldn't open /dev/ttyACM0.");
		std::cerr << "Couldn't open /dev/ttyACM0." << std::flush;
	}
	struct termios settings;
	tcgetattr(ttyFd, &settings);
	cfsetospeed(&settings, B115200);
	tcsetattr(ttyFd, TCSANOW, &settings);
	tcflush(ttyFd, TCOFLUSH);
	tty = fdopen(ttyFd, "rw");
	return true;
}

void setDrive(const char motor, const char direction, const float spd = 0.0f)
{
	std::stringstream ss;
	ss << 'd' << motor << direction;
	if(direction != MOTOR_STOP) {
		ss << std::hex << (int)(spd * 255);
	}
	ss << std::endl;
	const char* output = ss.str().c_str();
	fprintf(tty, "%s", output);
	ROS_INFO("%s", output);
}

void setRotation(const char motor, const char direction, const float spd = 0.0f)
{
	std::stringstream ss;
	ss << 'r' << motor << direction;
	if(direction != MOTOR_STOP) {
		ss << std::hex << (int)(spd * 255);
	}
	ss << std::endl;
	const char* output = ss.str().c_str();
	fprintf(tty, "%s", output);
	ROS_INFO("%s", output);
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
	char leftDir = leftSpd > 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
	char rightDir = rightSpd > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	// Map [-1, 1] -> [0, 1] as we've extracted the directional component
	leftSpd = leftSpd < 0 ? leftSpd * -1.0 : leftSpd;
	rightSpd = rightSpd < 0 ? rightSpd * -1.0 : rightSpd;
	leftSpd = min(1.0, max(0.0, leftSpd));
	rightSpd = min(1.0, max(0.0, rightSpd));
	// Apply!
	setDrive(MOTOR_LEFT, leftDir, leftSpd);
	setDrive(MOTOR_RIGHT, rightDir, rightSpd);

	// Wheel disc rotation
	double leftRotSpd = msg.angular.y;
	double rightRotSpd = msg.angular.y;
	char leftRotDir = leftRotSpd > 0 ? MOTOR_BACKWARD : MOTOR_FORWARD;
	char rightRotDir = rightRotSpd > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
	leftRotSpd = leftRotSpd < 0 ? leftRotSpd * -1.0 : leftRotSpd;
	rightRotSpd = rightRotSpd < 0 ? rightRotSpd * -1.0 : rightRotSpd;
	leftRotSpd = min(1.0, max(0.0, leftRotSpd));
	rightRotSpd = min(1.0, max(0.0, rightRotSpd));
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
setDrive(MOTOR_LEFT, MOTOR_FORWARD, 1.0);
	setDrive(MOTOR_RIGHT, MOTOR_STOP);
	setRotation(MOTOR_LEFT, MOTOR_STOP);
	setRotation(MOTOR_RIGHT, MOTOR_STOP);

	// This is correct - we're borrowing the turtle's topics
	//ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	odomPub = n.advertise<nav_msgs::Odometry>("odom", 50);
	odomBroadcaster = boost::make_shared<tf::TransformBroadcaster>();
	ros::Timer odoTimer = n.createTimer(ros::Duration(1.0/2.0/*2 Hz*/), publishOdometry);
	ROS_INFO("enginecontrol up and running.");
	ros::spin();
	fclose(tty);
	return 0;
}
