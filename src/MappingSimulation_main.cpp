#include "MappingSimulation.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "MappingSimulation");
	MappingSimulation ms;
	ros::spin();
	return 0;
}
