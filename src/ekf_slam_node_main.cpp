#include "EKF_SLAM_Node.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "EKF_SLAM_Node");
	EKF_SLAM_Node esn;
	ros::spin();
	return 0;
}