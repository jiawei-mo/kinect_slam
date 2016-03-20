#include "LandmarkExtractorNode.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LandmarkExtractorNode");
	LandmarkExtractorNode len;
	ros::spin();
	return 0;
}