#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
//#include "ArTimeToROSTime.h"
#include <stdio.h>
#include <cmath>
#include "kinect_slam/Pose2DMsg.h"
class Control
{
public:
	void pose_correction(double theta,double cheat_time);
private:
	ros::NodeHandle n;
};