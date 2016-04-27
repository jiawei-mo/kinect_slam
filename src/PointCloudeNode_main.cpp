#include "PointCloudNode.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudNode");
	PointCloudNode pcn;
	ros::spin();
	pcn.build_octomap(); // will this work?
	return 0;
}
