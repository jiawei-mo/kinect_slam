#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "EKF_SLAM.hpp"
#include "kinect_slam/EKFSLAMConfig.h"
#include "kinect_slam/PioneerVelControl.h"
#include "kinect_slam/LandmarkMsg.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

typedef boost::shared_ptr<kinect_slam::PioneerVelControl const> PioneerVelControlConstPtr;
typedef boost::shared_ptr<kinect_slam::LandmarkMsg const> LandmarkMsgConstPtr;

class EKF_SLAM_Node
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<kinect_slam::PioneerVelControl> vel_sub;
	message_filters::Subscriber<kinect_slam::LandmarkMsg> lmk_sub;
	message_filters::TimeSynchronizer<kinect_slam::PioneerVelControl, kinect_slam::LandmarkMsg> sync;

	boost::shared_ptr<EKF_SLAM> slam_ptr;
	dynamic_reconfigure::Server<kinect_slam::EKFSLAMConfig> server;
	dynamic_reconfigure::Server<kinect_slam::EKFSLAMConfig>::CallbackType f;

  double max_signature_threshold;
  double match_threshold;
  int new_landmark_threshold;

public:
	EKF_SLAM_Node();
	~EKF_SLAM_Node(){};
	void updateConfig(kinect_slam::EKFSLAMConfig &config, uint32_t level);
	void CtrlLmkCallback(const kinect_slam::PioneerVelControlConstPtr& ctrl, const kinect_slam::LandmarkMsgConstPtr& lmk);
};