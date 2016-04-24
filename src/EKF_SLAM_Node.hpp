#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "EKF_SLAM.hpp"
#include "kinect_slam/EKFSLAMConfig.h"
#include "nav_msgs/Odometry.h"
#include "kinect_slam/LandmarkMsg.h"
#include <geometry_msgs/TwistStamped.h>

typedef boost::shared_ptr<kinect_slam::LandmarkMsg const> LandmarkMsgConstPtr;

class EKF_SLAM_Node
{
private:
	ros::NodeHandle nh;
	ros::Subscriber vel_sub;
	ros::Subscriber lmk_sub;

	boost::shared_ptr<EKF_SLAM> slam_ptr;
	dynamic_reconfigure::Server<kinect_slam::EKFSLAMConfig> server;
	dynamic_reconfigure::Server<kinect_slam::EKFSLAMConfig>::CallbackType f;

  double max_signature_threshold;
  double match_threshold;
  double pre_time_stamp;

public:
	EKF_SLAM_Node();
	~EKF_SLAM_Node(){};
	void updateConfig(kinect_slam::EKFSLAMConfig &config, uint32_t level);
	//void CtrlCallback(const nav_msgs::Odometry::ConstPtr& ctrl);
	void CtrlCallback(const geometry_msgs::TwistStamped &ctrl);
	void LmkCallback(const kinect_slam::LandmarkMsgConstPtr& lmk);
};