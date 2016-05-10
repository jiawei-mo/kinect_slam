#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose2D.h>
//#include "ArTimeToROSTime.h"
#include <stdio.h>
#include <cmath>
#include <sensor_msgs/PointCloud.h>
#define PI 3.14159
class Control
{
public:
	bool pose_correction(double theta, int turn_flag);
	bool follow_wall(int flag, int step_flag,double distance);
	bool turn_left();
	bool turn_right();
	bool go_straight();
	bool check_pose(double theta);
	double compute_pose_correct(double theta);
private:
	ros::NodeHandle n;
	ros::Subscriber current_sonar;
	ros::Subscriber current_orientation;
	double current_left;
	double current_right;
	double current_theta;
	double lock;
    void sonarMeassageReceived(const sensor_msgs::PointCloud &msg);
    //void poseMeassageReceived(const geometry_msgs::PoseStamped &msg)
};