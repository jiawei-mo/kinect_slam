#include <ros/ros.h>
#include "parameter.hpp"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "Control.hpp"
#include "kinect_slam/LandmarkMsg.h"
#include <random>
#include <vector>
#include <Eigen/Core>
#include <math.h>
typedef boost::shared_ptr<kinect_slam::LandmarkMsg const> LandmarkMsgConstPtr;
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
    ros::Time turn_time;
    double current_theta;
    double current_EKF_theta;
    ros::Time follow_wall_time;
    bool action_lock;
    bool first_turn;
    bool pose_corrected;
    double avoid_wall;
    ros::Time current_time;
    ros::Publisher pub;
    ros::Publisher velocity;
    ros::Subscriber point_data;
    int turn_flag;
    bool follow_wall_flag;
   // double pre_sonar;
public:
	Control_Node();
	~Control_Node(){};
	void sonarMeassageReceived(const sensor_msgs::PointCloud &msg);
	void poseMeassageReceived(const geometry_msgs::PoseStamped &msg);
    void pointMeassageReceived(const kinect_slam::LandmarkMsgConstPtr &msg);
};
