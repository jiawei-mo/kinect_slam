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
#define OBSTACLE_FRONT 0.1 //2
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
	int correct_count;
	double distance_maintain;
    double correction_threshold;
    double turn_time;
    double correct_time;
    double current_theta;
<<<<<<< HEAD
    double pre_follow_wall_time;
    double follow_wall_count;
=======
>>>>>>> 7848c2a4f7a859934ccc927b2a578e8138c89a7b
   // double pre_sonar;
public:
	Control_Node();
	~Control_Node(){};
	void sonarMeassageReceived(const sensor_msgs::PointCloud &msg);
	void poseMeassageReceived(const geometry_msgs::Pose2D &msg);
};