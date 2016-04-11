#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "Control.hpp"
#define OBSTACLE_FRONT 2
#define OBSTACLE_SIDES 3
#define LEFT_AVAILABLE 3
#define LEFT_AVAILABLE_SIDES 4

class Control_Node
{
private:
  ros::NodeHandle nh;
	ros::Subscriber sonar;
	Control myCtrl;
	int turn_count;

public:
	Control_Node();
	~Control_Node(){};
	void sonarMeassageReceived(const sensor_msgs::PointCloud &msg);
};