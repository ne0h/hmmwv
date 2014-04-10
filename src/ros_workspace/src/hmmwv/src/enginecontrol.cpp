#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace ros;
using namespace std;

void velocityCallback(const geometry_msgs::Twist& msg) {
	ROS_INFO("linear: %f angular: %f", msg.linear.x, msg.angular.z);
}

int main(int argc, char **argv) {

	// init ros
	init(argc, argv, "enginecontrol");
	NodeHandle n;
	Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	
	// enter ros loop and wait for callbacks
	spin();

	return 0;
}
