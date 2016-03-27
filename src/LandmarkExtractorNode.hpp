#include <dynamic_reconfigure/server.h>
#include "HarrisDetector.hpp"
#include "BRIEF.hpp"
#include "kinect_slam/KinectSLAMConfig.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "kinect_slam/LandmarkMsg.h"

class LandmarkExtractorNode
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> img_sub;
	message_filters::Subscriber<sensor_msgs::Image> dep_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync;
	ros::Publisher landmark_pub;
	ros::Publisher pcl_pub;
	boost::shared_ptr<HarrisDetector> fd_ptr;
	boost::shared_ptr<BRIEF> de_ptr;
	dynamic_reconfigure::Server<kinect_slam::KinectSLAMConfig> server;
	dynamic_reconfigure::Server<kinect_slam::KinectSLAMConfig>::CallbackType f;

public:
	LandmarkExtractorNode();
	~LandmarkExtractorNode(){};
	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info);
	void updateConfig(kinect_slam::KinectSLAMConfig &config, uint32_t level);
};