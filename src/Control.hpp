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
	bool follow_wall(int flag);
	bool turn_left();
	bool turn_right();
	bool go_straight();
private:
	ros::NodeHandle n;
	bool lock;
};