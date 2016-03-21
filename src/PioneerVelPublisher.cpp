#include "ros/ros.h"
#include "kinect_slam/PioneerVelControl.h"
#include <stdlib.h>
#include <time.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pioneerVelPublisherNode");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<kinect_slam::PioneerVelControl>("control", 50);
	srand(time(NULL));

	kinect_slam::PioneerVelControl new_pvc;
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		new_pvc.left_vel = rand() % 10;
		new_pvc.right_vel = rand() % 10;
		pub.publish(new_pvc);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}