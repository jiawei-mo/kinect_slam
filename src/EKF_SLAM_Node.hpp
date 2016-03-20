#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "EKF_SLAM.hpp"
#include "kinect_slam/KinectSLAMConfig.h"
#include "kinect_slam/PioneerVelControl.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

typedef boost::shared_ptr<kinect_slam::PioneerVelControl const> PioneerVelControlConstPtr;

class EKF_SLAM_Node
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<kinect_slam::PioneerVelControl> vel_sub;
	message_filters::Subscriber<sensor_msgs::Image> lmk_sub;
	message_filters::TimeSynchronizer<kinect_slam::PioneerVelControl, sensor_msgs::Image> sync;

	boost::shared_ptr<EKF_SLAM> slam_ptr;
	dynamic_reconfigure::Server<kinect_slam::KinectSLAMConfig> server;
	dynamic_reconfigure::Server<kinect_slam::KinectSLAMConfig>::CallbackType f;

	double vertical_offset;
	double max_horizontal_threshold;
  double min_horizontal_threshold;
  double dist_threshold;

public:
	EKF_SLAM_Node();
	~EKF_SLAM_Node(){};
	void updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level);
	void CtrlLmkCallback(const kinect_slam::PioneerVelControlConstPtr& ctrl, const sensor_msgs::ImageConstPtr& lmk);
};