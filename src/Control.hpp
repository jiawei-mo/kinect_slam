#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
//#include "ArTimeToROSTime.h"
#include <stdio.h>
#include <cmath>
#include "kinect_slam/Pose2DMsg.h"
#define PI 3.14159
class Control
{
public:
	void pose_correction(double theta,double cheat_time);
	void follow_wall(int flag);
private:
	ros::NodeHandle n;
};