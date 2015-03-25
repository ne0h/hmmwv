#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>

using namespace std;

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
}

void publishOdometry(const ros::TimerEvent&) {
	// Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
	// Compute input values
	double dt = (currentTime - lastTime).toSec();
	double dx = (vx * cos(theta) - vy * sin(theta)) * dt * 1.164 /* m/s robot speed*/;
	double dy = (vx * sin(theta) - vy * cos(theta)) * dt * 1.164 /*m/s*/;
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
	odom.child_frame_id = "base_link";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odomQuat;

	//set the velocity
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vtheta;

	odomPub.publish(odom);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometrytest");
	ros::NodeHandle n;
	currentTime = ros::Time::now();
	lastTime = ros::Time::now();
	ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	odomPub = n.advertise<nav_msgs::Odometry>("odom", 50);
	odomBroadcaster = boost::make_shared<tf::TransformBroadcaster>();
	ros::Timer odoTimer = n.createTimer(ros::Duration(1.0/10.0/*10 Hz*/), publishOdometry);
	ROS_INFO("odometrytest up and running.");
	ros::spin();
	return 0;
}
