#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include "Control.hpp"
#define OBSTACLE_FRONT 2.5 //2
#define OBSTACLE_SIDES 3
#define LEFT_AVAILABLE 5 //3
#define LEFT_AVAILABLE_SIDES 3

class Control_Node
{
private:
    ros::NodeHandle nh;
	ros::Subscriber sonar;
	ros::Subscriber pose_correct;
	Control myCtrl;
	int turn_count;
	double distance_maintain;
    double correction_threshold;
    double turn_time;
    double current_theta;

   // double pre_sonar;
public:
	Control_Node();
	~Control_Node(){};
	void sonarMeassageReceived(const sensor_msgs::PointCloud &msg);
	void poseMeassageReceived(const kinect_slam::Pose2DMsg &msg);
};