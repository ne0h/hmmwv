#include "gpio.hpp"
#include "engine.hpp"

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>

using namespace std;

GPIO gpio;

// 						enable		direction		speed
Engine driveLeft(&gpio, GPIO::P9_31, GPIO::P9_21, GPIO::P9_14);
Engine driveRight(&gpio, GPIO::P8_10, GPIO::P8_12, GPIO::P8_13);
Engine rotatorLeft(&gpio, GPIO::P9_24, GPIO::P9_26, GPIO::P9_16);
Engine rotatorRight(&gpio, GPIO::P8_17, GPIO::P8_15, GPIO::P8_19);

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

void velocityCallback(const geometry_msgs::Twist& msg) {
	//ROS_INFO("%f", msg.linear.x);

	// Store odometry input values
	lastTime = currentTime;
	currentTime = ros::Time::now();
	vx = msg.linear.x;
	vy = msg.linear.y;
	vtheta = msg.angular.z * 10; // absolutely uneducated guess

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

	// This is correct - we're borrowing the turtle's topics
	ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	odomPub = n.advertise<nav_msgs::Odometry>("odom", 50);
	odomBroadcaster = boost::make_shared<tf::TransformBroadcaster>();
	ros::Timer odoTimer = n.createTimer(ros::Duration(1.0/2.0/*2 Hz*/), publishOdometry);
	ROS_INFO("enginecontrol up and running.");
	ros::spin();
	return 0;
}
