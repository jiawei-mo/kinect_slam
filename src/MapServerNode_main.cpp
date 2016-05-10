#include "MapServerNode.hpp"
#include "ros/ros.h"
int main(int argc, char** argv)
{
	ros::init(argc, argv, "MapServerNode");
	// std::cout<<"pos1";
	MapServerNode msn;
	// std::cout<<"pos2";
	ros::spin();
	return 0;
}