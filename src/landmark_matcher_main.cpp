#include "LandmarkMatcherNode.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LandmarkMatcherNode");
	LandmarkMatcherNode lmn;
	ros::spin();
	return 0;
}