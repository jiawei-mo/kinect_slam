#include <dynamic_reconfigure/server.h>
#include "HarrisDetector.hpp"
#include "BRIEF.hpp"
#include "kinect_slam/LandmarkExtractorConfig.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "kinect_slam/LandmarkMsg.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> KinectSyncPolicy;

class LandmarkExtractorNode
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> img_sub;
	message_filters::Subscriber<sensor_msgs::Image> dep_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
	message_filters::Synchronizer<KinectSyncPolicy> sync;
	ros::Publisher landmark_pub;
	ros::Publisher pcl_current_frame_pub;
	ros::Publisher raw_point_pub;
	boost::shared_ptr<HarrisDetector> fd_ptr;
	boost::shared_ptr<BRIEF> de_ptr;
	dynamic_reconfigure::Server<kinect_slam::LandmarkExtractorConfig> server;
	dynamic_reconfigure::Server<kinect_slam::LandmarkExtractorConfig>::CallbackType f;

	double MIN_DEPTH;
	double MAX_DEPTH;

public:
	LandmarkExtractorNode();
	~LandmarkExtractorNode(){};
	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info);
	void updateConfig(kinect_slam::LandmarkExtractorConfig &config, uint32_t level);
};
