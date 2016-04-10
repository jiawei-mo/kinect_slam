#include "Control_Node.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Control_Node");
	Control_Node cn;
	ros::spin();
	return 0;
}